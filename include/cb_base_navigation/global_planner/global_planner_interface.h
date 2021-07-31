/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_GLOBAL_PLANNER_INTERFACE_
#define cb_global_planner_GLOBAL_PLANNER_INTERFACE_

#include "global_planner_plugin.h"
#include "visualization.h"

//! Messages + Services
#include <cb_base_navigation_msgs/PositionConstraint.h>
#include <cb_base_navigation_msgs/OrientationConstraint.h>
#include <cb_base_navigation_msgs/LocalPlannerActionGoal.h>
#include <cb_base_navigation_msgs/CheckPlan.h>
#include <cb_base_navigation_msgs/GetPlan.h>

#include <geometry_msgs/PoseStamped.h>

#include <pluginlib/class_loader.h>

#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>

namespace costmap_2d {
class Costmap2DROS;
}

namespace tf2_ros {
class Buffer;
}

namespace cb_global_planner {

class GlobalPlannerInterface {

public:

    GlobalPlannerInterface(costmap_2d::Costmap2DROS* costmap, tf2_ros::Buffer* tf);

    ~GlobalPlannerInterface();

private:

    //! Connections to the outside world
    ros::ServiceServer get_plan_srv_, check_plan_srv_;
    bool getPlan(cb_base_navigation_msgs::GetPlanRequest& req, cb_base_navigation_msgs::GetPlanResponse& resp);
    bool checkPlan(cb_base_navigation_msgs::CheckPlanRequest& req, cb_base_navigation_msgs::CheckPlanResponse& resp);

    //! Pose callback and publisher
    ros::Subscriber pose_sub_;
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
    ros::Publisher plan_pub_;

    //! Planners + loaders
    boost::shared_ptr<GlobalPlannerPlugin> global_planner_;
    pluginlib::ClassLoader<GlobalPlannerPlugin> gp_loader_;

    //! Frame names + Tranforms
    std::string robot_base_frame_, global_frame_;
    tf2_ros::Buffer* tf_;

    //! Costmaps
    costmap_2d::Costmap2DROS* costmap_;

    //! Visualization
    Visualization vis_;

};

}

#endif
