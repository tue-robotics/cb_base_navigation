#include "cb_base_navigation/local_planner/local_planner_interface.h"

#include <tue/profiling/ros/profile_publisher.h>
#include <tue/profiling/scoped_timer.h>

#include <boost/thread.hpp>

using namespace cb_planner_msgs_srvs;

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

    // Topic sub
    topic_sub_ = gh.subscribe("action_server/goal", 1, &LocalPlannerInterface::topicGoalCallback, this);

    ROS_INFO_STREAM("Local planner of type: '" << local_planner << "' initialized.");

    // Start the controller thread
    controller_thread_ = new boost::thread(boost::bind(&LocalPlannerInterface::controllerThread, this));
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
        ROS_ERROR("LPI: Local costmap map update frequency should be 0, it is now: %2f Hz. This is not allowed by the Local Planner Interface!",costmap_->getMapUpdateFrequency());
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
            tue::ScopedTimer timer(profiler, "publishCostmap()");
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

void LocalPlannerInterface::doSomeMotionPlanning()
{
    boost::unique_lock<boost::mutex> lock(goal_mtx_);

    if (!isGoalSet()) return;

    // 1) Update the end goal orientation due to the orientation constraint, if updated, set new plan
    if (updateEndGoalOrientation())
        local_planner_->setPlan(goal_.plan);

    // 2) Check if we are already there
    if (local_planner_->isGoalReached()) {
        ROS_INFO("LPI: Local planner arrived at the goal position/orientation; clearing plan...");
        action_server_->setSucceeded();
        goal_.plan.clear();
        return;
    }

    // 3) Compute and publish velocity commands to base, check if not blocked
    geometry_msgs::Twist tw;
    bool blocked = !local_planner_->computeVelocityCommands(tw);
    vel_pub_.publish(tw);

    // 4) Publish some feedback to via the action_server
    feedback_.dtg = 0.0; //! TODO: NY implemented
    feedback_.blocked = blocked;
    action_server_->publishFeedback(feedback_);
}

bool LocalPlannerInterface::updateEndGoalOrientation()
{
    if (!isGoalSet()) return false;

    // Request the desired transform from constraint frame to map
    tf::StampedTransform constraint_to_world_tf;
    try {
        tf_->lookupTransform(costmap_->getGlobalFrameID(), goal_.orientation_constraint.frame, ros::Time(0), constraint_to_world_tf);
        feedback_.frame_exists = true;
    } catch(tf::TransformException& ex) {
        ROS_ERROR("LPI: Failed to get robot orientation constraint frame");
        feedback_.frame_exists = false;
        return false;
    }

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
