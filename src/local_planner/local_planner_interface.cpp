#include "cb_base_navigation/local_planner/local_planner_interface.h"

#include <tue/profiling/ros/profile_publisher.h>
#include <tue/profiling/scoped_timer.h>

#include <boost/thread.hpp>
#include <base_local_planner/goal_functions.h>

// Querying world model (ED)
#include <ed_msgs/SimpleQuery.h>

using namespace cb_base_navigation_msgs;

namespace cb_local_planner {

LocalPlannerInterface::~LocalPlannerInterface()
{
    // Clean things up
    local_planner_.reset();

    delete tf_;
    delete action_server_;

    controller_thread_->interrupt();
    controller_thread_->join();
}

LocalPlannerInterface::LocalPlannerInterface(costmap_2d::Costmap2DROS* costmap) :
    lp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    tf_(new tf::TransformListener(ros::Duration(10))),
    costmap_(costmap)
{
    ros::NodeHandle gh;
    ros::NodeHandle nh("~");

    // Parameter setup
    std::string local_planner;
    nh.param("local_planner", local_planner, std::string("dwa_local_planner/DWAPlannerROS"));
    nh.param("robot_base_frame", robot_base_frame_, std::string("/base_link"));;
    nh.param("global_frame", global_frame_, std::string("/map"));
    nh.param("controller_frequency", controller_frequency_, 20.0);

    // ROS Publishers
    vel_pub_ = gh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

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
        local_planner_->initialize(lp_loader_.getName(local_planner), tf_, costmap_);
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

    ROS_INFO_STREAM("Local planner of type: '" << local_planner << "' initialized.");

    // Start the controller thread
    controller_thread_ = new boost::thread(boost::bind(&LocalPlannerInterface::controllerThread, this));

    ros::NodeHandle n;
    ed_client_ = n.serviceClient<ed_msgs::SimpleQuery>("ed/simple_query");
}

void LocalPlannerInterface::topicGoalCallback(const LocalPlannerActionGoalConstPtr& goal)
{
    boost::unique_lock<boost::mutex> lock(goal_mtx_); // goal is altered

    ROS_INFO("Incoming topic plan.");

    goal_ = goal->goal;

    if (isGoalSet()) {
        updateEndGoalOrientation();
        local_planner_->setPlan(goal_.plan);
    } else {
        ROS_ERROR("Received a plan of length 0, is something wrong?");
    }
}

void LocalPlannerInterface::actionServerSetPlan()
{
    boost::unique_lock<boost::mutex> lock(goal_mtx_); // goal is altered

    ROS_INFO("Incoming actionlib plan.");

    goal_ = *action_server_->acceptNewGoal();

    if (isGoalSet()) {
        updateEndGoalOrientation();
        local_planner_->setPlan(goal_.plan);
    } else {
        ROS_ERROR("Received a plan of length 0, is something wrong?");
    }
}

void LocalPlannerInterface::actionServerPreempt()
{
    boost::unique_lock<boost::mutex> lock(goal_mtx_); // goal is altered

    action_server_->setPreempted();
    goal_.plan.clear();
}

void LocalPlannerInterface::controllerThread()
{
	//TODO segfault at startup, quick fix
	ros::Duration(5.0).sleep();

    tue::ProfilePublisher profile_pub;
    tue::Profiler profiler;
    profile_pub.initialize(profiler);

    ros::Rate r(controller_frequency_);

    ROS_INFO_STREAM("LPI: Started local planner thread @ " << controller_frequency_ << " hz!");

    if (costmap_->getMapUpdateFrequency() > 0)
    {
        ROS_ERROR("LPI: Local costmap map update frequency should be 0, it is now: %2f Hz. This is not allowed by the Local Planner Interface!", costmap_->getMapUpdateFrequency());
        ROS_ERROR("LPI:     [[  Shutting down ...  ]]     ");
        exit(1);
    }

    while (ros::ok())
    {
        tue::ScopedTimer timer(profiler, "total");

        {
            tue::ScopedTimer timer(profiler, "updateMap()");
            costmap_->updateMap();
        }

        {
            tue::ScopedTimer timer(profiler, "publishCostmap()");
            costmap_->getPublisher()->publishCostmap();
        }

        {
            tue::ScopedTimer timer(profiler, "doSomeMotionPlanning()");
            doSomeMotionPlanning();
        }

        {
            tue::ScopedTimer timer(profiler, "rateCheck()");
            static ros::Time t_last_rate_met = ros::Time::now();

            if (r.cycleTime() < r.expectedCycleTime())
                t_last_rate_met = ros::Time::now();

            if ((ros::Time::now() - t_last_rate_met).toSec() > 1.0) // When we do not meet the rate, print it every 1 second
            {
                ROS_WARN_STREAM("LPI: Local planner rate of " << 1.0 / r.expectedCycleTime().toSec() << " hz not met. " << profiler);
                t_last_rate_met = ros::Time::now();
            }
        }

        {
            tue::ScopedTimer timer(profiler, "sleep()");
            r.sleep();
        }

        profile_pub.publish();
    }
}

void prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan){
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    while(it != plan.end()){
      const geometry_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.getOrigin().x() - w.pose.position.x;
      double y_diff = global_pose.getOrigin().y() - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < .5){
        it = plan.erase(it);
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
    }
  }

double getDistance(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    double distance = 0;
    if (plan.size() > 1)
    {
        for (unsigned int i = 1; i < plan.size(); ++i)
            distance+=hypot(plan[i].pose.position.x-plan[i-1].pose.position.x, plan[i].pose.position.y-plan[i-1].pose.position.y);
    }
    return distance;
}

bool getBlockedPoint(const std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2D* costmap, geometry_msgs::Point& p)
{
    for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin(); it != plan.end(); ++it)
    {
        unsigned int mx, my;
        if (costmap->worldToMap(it->pose.position.x, it->pose.position.y, mx, my))
        {
            if (costmap->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                p = it->pose.position;
                return true;
            }
        }
    }
    return false;
}

void LocalPlannerInterface::doSomeMotionPlanning()
{
    boost::unique_lock<boost::mutex> lock(goal_mtx_);

    if (!isGoalSet() || !costmap_->getRobotPose(global_pose_)) return;

    // 1) Update the end goal orientation due to the orientation constraint, if updated, set new plan
    if (updateEndGoalOrientation())
        local_planner_->setPlan(goal_.plan);

    // 2) Check if we are already there
    if (local_planner_->isGoalReached()) {
        geometry_msgs::Twist tw;
        vel_pub_.publish(tw);
        ROS_INFO("LPI: Local planner arrived at the goal position/orientation; clearing plan...");
        action_server_->setSucceeded();
        goal_.plan.clear();
        return;
    }

    // 3) Compute and publish velocity commands to base, check if not blocked
    geometry_msgs::Twist tw;
    feedback_.blocked = !local_planner_->computeVelocityCommands(tw);
    vel_pub_.publish(tw);

    // 4) Get the pruned plan
    std::vector<geometry_msgs::PoseStamped> pruned_plan = goal_.plan;
    prunePlan(global_pose_, pruned_plan);

    // 4) Publish some feedback to via the action_server
    feedback_.dtg = getDistance(pruned_plan);
    if (feedback_.dtg < 1)
        feedback_.dtg = base_local_planner::getGoalPositionDistance(global_pose_, pruned_plan.end()->pose.position.x, pruned_plan.end()->pose.position.y);
    if (feedback_.blocked)
        feedback_.blocked = getBlockedPoint(pruned_plan, costmap_->getCostmap(), feedback_.point_blocked);
    action_server_->publishFeedback(feedback_);
}

bool LocalPlannerInterface::updateEndGoalOrientation()
{
    if (!isGoalSet()) return false;

    tf::Transform constraint_to_world_tf;

    if (goal_.orientation_constraint.frame == "/map" || goal_.orientation_constraint.frame == "map")
    {
        constraint_to_world_tf = tf::Transform::getIdentity();
    }
    else
    {
        ed_msgs::SimpleQuery ed_query;
        ed_query.request.id = goal_.orientation_constraint.frame;

        if (!ed_client_.call(ed_query))
        {
            ROS_ERROR("LPI: Failed to get robot orientation constraint frame: ED could not be queried.");
            return false;
        }

        if (ed_query.response.entities.empty())
        {
            ROS_ERROR_STREAM("LPI: Failed to get robot orientation constraint frame: ED returns 'no such entity'. ("
                             << goal_.orientation_constraint.frame << ").");
            return false;
        }

        const ed_msgs::EntityInfo& e_info = ed_query.response.entities.front();

        tf::Transform world_to_constraint_tf;
        tf::poseMsgToTF(e_info.pose, world_to_constraint_tf);
        constraint_to_world_tf = world_to_constraint_tf;
    }

//    // Request the desired transform from constraint frame to map
//    tf::StampedTransform constraint_to_world_tf;
//    try {
//        tf_->lookupTransform(costmap_->getGlobalFrameID(), goal_.orientation_constraint.frame, ros::Time(0), constraint_to_world_tf);
//        feedback_.frame_exists = true;
//    } catch(tf::TransformException& ex) {
//        ROS_ERROR("LPI: Failed to get robot orientation constraint frame");
//        feedback_.frame_exists = false;
//        return false;
//    }

    // Get end and look at point
    tf::Point look_at, end_point;
    tf::pointMsgToTF(goal_.orientation_constraint.look_at, look_at);
    tf::pointMsgToTF(goal_.plan.back().pose.position, end_point);

    // Get difference
    tf::Point diff = constraint_to_world_tf*look_at - end_point;

    // Calculate current and new orientation
    geometry_msgs::Quaternion& q = goal_.plan.back().pose.orientation;
    tf::Quaternion new_quat = tf::createQuaternionFromYaw(atan2(diff.y(),diff.x()) + goal_.orientation_constraint.angle_offset);

    // Check if the end goal orientation is already set
    bool changed = (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0);

    if (!changed)
    {
        tf::Quaternion current_quat;
        tf::quaternionMsgToTF(goal_.plan.back().pose.orientation, current_quat);
        changed = abs(current_quat.dot(new_quat) < 1-1e-6);
    }

    // Check if the orientation has changed
    if (changed)
    {
        // Set new orientation
        tf::quaternionTFToMsg(new_quat, q);

        // Visualize the end pose
        vis.publishGoalPoseMarker(goal_.plan.back());

        return true;
    }

    return false;
}

}
