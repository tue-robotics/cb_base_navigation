/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_local_planner_LOCAL_PLANNER_INTERFACE_
#define cb_local_planner_LOCAL_PLANNER_INTERFACE_

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <actionlib/server/simple_action_server.h>

#include <cb_planner_msgs_srvs/LocalPlannerAction.h>
#include "visualization.h"

#include <actionlib/client/action_client.h>

#include <tue/profiling/profiler.h>

#define PI 3.14159265

using namespace cb_planner_msgs_srvs;

namespace cb_local_planner {

class LocalPlannerInterface {

public:

    LocalPlannerInterface(costmap_2d::Costmap2DROS* costmap);
    ~LocalPlannerInterface();

private:

    boost::mutex goal_mtx_;
    boost::thread* controller_thread_;

    void controllerThread();
    void doSomeMotionPlanning();

    inline bool isGoalSet() { return goal_.plan.size() > 0; }

    //! ROS Communication
    ros::Publisher vel_pub_;
    ros::Subscriber topic_sub_;
    actionlib::SimpleActionServer<LocalPlannerAction>* action_server_;

    //! topic goal cb
    void topicGoalCallback(const LocalPlannerActionGoalConstPtr &goal);

    //! World model client
    ros::ServiceClient ed_client_;

    //! Action Server stuff
    void actionServerSetPlan();
    void actionServerPreempt();
    LocalPlannerGoal goal_;
    LocalPlannerFeedback feedback_;

    //! Planners + loaders
    boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> lp_loader_;
    double controller_frequency_;

    //! Frame names + Tranforms
    std::string robot_base_frame_, global_frame_;
    tf::TransformListener* tf_;
    tf::Stamped<tf::Pose> global_pose_;

    //! Costmaps
    costmap_2d::Costmap2DROS* costmap_;

    //! Helper functions
    bool updateEndGoalOrientation();

    //! Visualization
    Visualization vis;

};

}

#endif
