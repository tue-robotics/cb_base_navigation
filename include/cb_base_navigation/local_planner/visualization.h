/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_local_planner_VISUALIZATION_H_
#define cb_local_planner_VISUALIZATION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

namespace cb_local_planner
{

/**
 * @class Class with visualization functions for the local planner interface.
 * @brief Implements some visualziation functions (ROS vis messages)
 */
class Visualization
{

public:
    /**
     * @brief  Constructor for the Visualization object (inits the publishers)
     */
    Visualization();

    /**
     * @brief  Publishes goal pose marker
     * @param  goal goal position and orientation
     * @param  frame coordinate frame
     */
    void publishGoalPoseMarker(const geometry_msgs::PoseStamped& goal, const std::string& frame = "/map");

private:
    ros::Publisher goal_pose_marker_pub_;

};

}
#endif

