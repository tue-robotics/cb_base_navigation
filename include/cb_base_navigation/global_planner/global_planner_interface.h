/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_GLOBAL_PLANNER_INTERFACE_
#define cb_global_planner_GLOBAL_PLANNER_INTERFACE_

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "visualization.h"
#include "global_planner_plugin.h"

//! Messages + Services
#include <cb_planner_msgs_srvs/PositionConstraint.h>
#include <cb_planner_msgs_srvs/OrientationConstraint.h>
#include <cb_planner_msgs_srvs/LocalPlannerActionGoal.h>
#include <cb_planner_msgs_srvs/CheckPlan.h>
#include <cb_planner_msgs_srvs/GetPlan.h>

namespace cb_global_planner {

class GlobalPlannerInterface {

public:

    GlobalPlannerInterface(costmap_2d::Costmap2DROS& costmap);
    ~GlobalPlannerInterface();

private:

    //! Connections to the outside world
    ros::ServiceServer get_plan_srv_, check_plan_srv_;
    bool getPlan(GetPlanRequest& req, GetPlanResponse& resp);
    bool checkPlan(CheckPlanRequest& req, CheckPlanResponse& resp);

    //! Pose callback and publisher
    ros::Subscriber pose_sub_;
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose);
    ros::Publisher plan_pub_;

    //! Planners + loaders
    boost::shared_ptr<GlobalPlannerPlugin> global_planner_;
    pluginlib::ClassLoader<GlobalPlannerPlugin> gp_loader_;

    //! Frame names + Tranforms
    std::string robot_base_frame_, global_frame_;
    tf::TransformListener* tf_;
    tf::Stamped<tf::Pose> global_pose_;

    //! Costmaps
    costmap_2d::Costmap2DROS& costmap_;

    //! Visualization
    Visualization vis_;

};

}

#endif
