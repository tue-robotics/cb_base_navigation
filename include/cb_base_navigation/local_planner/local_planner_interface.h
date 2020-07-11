/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_local_planner_LOCAL_PLANNER_INTERFACE_
#define cb_local_planner_LOCAL_PLANNER_INTERFACE_

#include "visualization.h"

#include <actionlib/client/action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/shared_ptr.hpp>

#include <cb_base_navigation_msgs/LocalPlannerAction.h>

#include <geometry_msgs/PoseStamped.h>

#include <pluginlib/class_loader.h>

#include <tf2/utils.h>

#include <memory>
#include <thread>

namespace costmap_2d {
class Costmap2DROS;
}

namespace nav_core {
class BaseLocalPlanner;
}

namespace tf2_ros {
class Buffer;
}

namespace cb_local_planner {

class LocalPlannerInterface {

public:

    LocalPlannerInterface(costmap_2d::Costmap2DROS* costmap, tf2_ros::Buffer* tf);

    ~LocalPlannerInterface();

private:

    boost::mutex goal_mtx_;
    std::unique_ptr<std::thread> controller_thread_;

    void controllerThread();
    void doSomeMotionPlanning();

    inline bool isGoalSet() { return goal_.plan.size() > 0; }

    //! ROS Communication
    ros::Publisher vel_pub_;
    ros::Subscriber topic_sub_;
    std::unique_ptr<actionlib::SimpleActionServer<cb_base_navigation_msgs::LocalPlannerAction> > action_server_;

    //! topic goal cb
    void topicGoalCallback(const cb_base_navigation_msgs::LocalPlannerActionGoalConstPtr &goal);

    //! World model client
    ros::ServiceClient ed_client_;

    //! Action Server stuff
    void actionServerSetPlan();
    void actionServerPreempt();
    cb_base_navigation_msgs::LocalPlannerGoal goal_;
    cb_base_navigation_msgs::LocalPlannerFeedback feedback_;

    //! Planners + loaders
    boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> lp_loader_;
    double controller_frequency_;

    //! Frame names + Tranforms
    std::string robot_base_frame_, global_frame_;
    tf2_ros::Buffer* tf_;
    geometry_msgs::PoseStamped global_pose_;

    //! Costmaps
    costmap_2d::Costmap2DROS* costmap_;

    //! Helper functions
    bool updateEndGoalOrientation();

    //! Visualization
    Visualization vis;

};

}

#endif
