#include "cb_base_navigation/local_planner/local_planner_interface.h"

using namespace cb_planner_msgs_srvs;

namespace cb_local_planner {

LocalPlannerInterface::~LocalPlannerInterface()
{
}

LocalPlannerInterface::LocalPlannerInterface(costmap_2d::Costmap2DROS& costmap) :
    lp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    tf_(new tf::TransformListener(ros::Duration(10))),
    costmap_(costmap)
{
    ros::NodeHandle nh("~");

    // Parameter setup
    std::string global_planner, local_planner;
    nh.param("local_planner", local_planner, std::string("cb_local_planner/AmigoLocalPlanner"));
    nh.param("robot_base_frame", robot_base_frame_, std::string("/amigo/base_link"));;
    nh.param("global_frame", global_frame_, std::string("/map"));

    // ROS Publishers
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/amigo/base/references", 1);

    // Initialize the local planner
    try {
        if(!lp_loader_.isClassAvailable(local_planner)){
            std::vector<std::string> classes = lp_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i){
                if(local_planner == lp_loader_.getName(classes[i])){
                    local_planner = classes[i]; break;
                }
            }
        }
        local_planner_ = lp_loader_.createInstance(local_planner);
        local_planner_->initialize(lp_loader_.getName(local_planner), tf_, &costmap_);
    } catch (const pluginlib::PluginlibException& ex)
    {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
        exit(0);
    }

    // Construct action server
    action_server_ = new actionlib::SimpleActionServer<LocalPlannerAction>(nh, "action_server", false);
    action_server_->registerGoalCallback(boost::bind(&LocalPlannerInterface::actionServerSetPlan, this));
    action_server_->registerPreemptCallback(boost::bind(&LocalPlannerInterface::actionServerPreempt, this));
    action_server_->start();

    // Head reference
    head_ref_ac_ = new actionlib::ActionClient<head_ref::HeadReferenceAction>("/HeadReference");

    // Topic sub
    topic_sub_ = nh.subscribe("action_server/goal", 1, &LocalPlannerInterface::topicGoalCallback, this);

    ROS_INFO_STREAM("Local planner of type: '" << local_planner << "' initialized. Ready to rumble!");
}

void LocalPlannerInterface::topicGoalCallback(const LocalPlannerActionGoalConstPtr& goal)
{
    ROS_INFO("Incoming topic plan.");

    goal_ = goal->goal;
    goal_.plan.back().pose.orientation.w = 1.0; // for normalization

    if (isGoalSet()) {
        local_planner_->setPlan(goal_.plan);
    } else {
        ROS_ERROR("Received a plan of length 0, is something wrong?");
    }
}

void LocalPlannerInterface::actionServerSetPlan()
{
    ROS_INFO("Incoming actionlib plan.");

    goal_ = *action_server_->acceptNewGoal();
    goal_.plan.back().pose.orientation.w = 1.0; // for normalization

    if (isGoalSet()) {
        local_planner_->setPlan(goal_.plan);
    } else {
        ROS_ERROR("Received a plan of length 0, is something wrong?");
    }
}

void LocalPlannerInterface::actionServerPreempt()
{
    action_server_->setPreempted();
    goal_.plan.clear();
}

void LocalPlannerInterface::doSomeMotionPlanning()
{
    if (!isGoalSet()) return;

    // 1) Update the end goal orientation due to the orientation constraint
    bool valid = updateEndGoalOrientation();

    // 2) Check if we are already there
    if (local_planner_->isGoalReached()) {
        ROS_INFO("Local planner arrived at the goal position/orientation; clearing plan...");
        action_server_->setSucceeded();
        goal_.plan.clear();
        return;
    }

    // 3) Check if we're not stuck (local minimum)
    //! TODO

    // 4) If we're not there and we're not stuck: compute and publish velocity commands to base
    geometry_msgs::Twist tw;

    local_planner_->computeVelocityCommands(tw);
    vel_pub_.publish(tw);

    // 5) Publish some feedback to via the action_server
    feedback_.dtg = 0.0; //! TODO: NY implemented
    feedback_.frame_exists = valid;
    action_server_->publishFeedback(feedback_);

    // 6) Look in the heading direction
    generateHeadReference(tw);
}

bool LocalPlannerInterface::updateEndGoalOrientation()
{
    if (!isGoalSet()) return false;

    // Request the desired transform from constraint frame to map
    tf::StampedTransform constraint_to_world_tf;
    try {
        tf_->lookupTransform(costmap_.getGlobalFrameID(), goal_.orientation_constraint.frame, ros::Time(0), constraint_to_world_tf);
    } catch(tf::TransformException& ex) {
        ROS_ERROR("Failed to get robot orientation constraint frame");
        //action_server_->setAborted();
        return false;
    }

    // Get end and look at point
    tf::Point look_at, end_point;
    tf::pointMsgToTF(goal_.orientation_constraint.look_at, look_at);
    tf::pointMsgToTF(goal_.plan.back().pose.position, end_point);

    // Get difference
    tf::Point diff = constraint_to_world_tf*look_at - end_point;

    // Calculate current and new orientation
    tf::Quaternion new_quat, current_quat;
    tf::quaternionMsgToTF(goal_.plan.back().pose.orientation, current_quat);
    new_quat = tf::createQuaternionFromYaw(atan2(diff.y(),diff.x()) + goal_.orientation_constraint.angle_offset);

    // Check if the orientation has changed
    if (abs(current_quat.dot(new_quat) > 1-1e-6))
    {
        // Set new orientation
        tf::quaternionTFToMsg(new_quat, goal_.plan.back().pose.orientation);

        // Update the plan
        local_planner_->setPlan(goal_.plan);

        // Visualize the end pose
        vis.publishGoalPoseMarker(goal_.plan.back());
    }

    return true;
}

void LocalPlannerInterface::generateHeadReference(const geometry_msgs::Twist& cmd)
{
    // send a goal to the action
    head_ref::HeadReferenceGoal goal;
    goal.target_point.header.frame_id = costmap_.getBaseFrameID();
    goal.target_point.header.stamp = ros::Time::now();

    double dt = 3; //! TODO: Hardcoded values in this function
    double x = dt * (cmd.linear.x * cos(cmd.angular.z) + cmd.linear.y * cos(PI/2 + cmd.angular.z));
    double y = dt * (cmd.linear.x * sin(cmd.angular.z) + cmd.linear.y * sin(PI/2 + cmd.angular.z));

    double th = atan2(y, x);

    //! When only turning
    if (cmd.linear.y*cmd.linear.y+cmd.linear.x*cmd.linear.x < 0.1*0.1)
        if (cmd.angular.z > 0)
            th = .2*PI;
        else if (cmd.angular.z < 0)
            th = -.2*PI;

    //! Limit -.5*PI till .5*PI
    th = std::max( -.5*PI , std::min (th, .5*PI) );

    //! At least 1 m
    double r = std::max(1.0 , y*y+x*x );

    goal.target_point.point.x = cos(th) * r;
    goal.target_point.point.y = sin(th) * r;
    goal.target_point.point.z = 0;

    goal.goal_type = head_ref::HeadReferenceGoal::LOOKAT;

    goal.priority = 5;

    goal.pan_vel = .3;
    goal.tilt_vel = .3;

    goal.end_time = ros::Time::now().toSec() + 1;

    // Update the goal handle
    head_ref_gh_ = head_ref_ac_->sendGoal(goal);
}

}
