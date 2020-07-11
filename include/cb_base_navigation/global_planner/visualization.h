/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_VISUALIZATION_H_
#define cb_global_planner_VISUALIZATION_H_

#include <geometry_msgs/PoseStamped.h>

#include <ros/publisher.h>

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace cb_global_planner
{

/**
 * @class Class with visualization functions for the global planner interface.
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
     * @brief  Publishes a specified plan
     * @param  plan plan to be published
     * @param  frame coordinate frame
     */
    void publishGlobalPlanMarker(const std::vector<geometry_msgs::PoseStamped>& plan, const std::string& frame = "map");

    /**
     * @brief  Publishes a specified plan with poses
     * @param  plan plan to be published
     * @param  frame coordinate frame
     */
    void publishGlobalPlanMarkerArray(const std::vector<geometry_msgs::PoseStamped>& plan, const std::string& frame = "map");

    /**
     * @brief  Publishes the goal area
     * @param  positions set of points which meet the end goal position constraints
     * @param  frame coordinate frame
     */
    void publishGoalPositionsMarker(const std::vector<tf2::Vector3>& positions, const std::string& frame = "map");

private:
    ros::Publisher global_plan_marker_array_pub_, global_plan_marker_pub_, goal_positions_marker_pub_;

};

}
#endif

