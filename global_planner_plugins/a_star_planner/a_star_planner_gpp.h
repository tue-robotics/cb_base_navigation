/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_ASTARPLANNER_GPP_H_
#define cb_global_planner_ASTARPLANNER_GPP_H_

#include <ros/ros.h>

#include "a_star_planner.h"
#include "cb_base_navigation/global_planner/global_planner_plugin.h"
#include "cb_base_navigation/global_planner/constraint_evaluator.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_datatypes.h>

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace cb_global_planner {

/**
 * @class AStar Constrained based GlobalPlannerPlugin
 * @brief Implementation of the A* Algorithm as constrained based global planner plugin.
 */
class AStarPlannerGPP : public GlobalPlannerPlugin
{

public:
	/**
     * @brief  Constructor for the AStarPlannerGPP object
	 */
    AStarPlannerGPP();

    ~AStarPlannerGPP();

	/**
     * @brief  Initialization function for the AStarPlannerGPP object
	 * @param  name The name of this planner
     * @param  tf Pointer to the tf listerner
	 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
	 */
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap_ros);

    /**
     * @brief Given a set of goal constraints, compute a plan
     * @param start The start pose
     * @param position_constraint Position contraints on the goal position
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const tf::Stamped<tf::Pose>& start, const PositionConstraint& position_constraint, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<tf::Point>& goal_positions);

    /**
     * @brief  Initialization function for the BaseGlobalPlanner
     * @param  name The name of this planner
     * @param  tf tf_listener
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

private:

    costmap_2d::Costmap2DROS* global_costmap_ros_;
	AStarPlanner* planner_;
    tf::TransformListener* tf_;
    bool initialized_;

    bool constraintChanged(PositionConstraint c) { return (position_constraint_.constraint != c.constraint || position_constraint_.frame != c.frame); }

    bool updateConstraintPositionsInConstraintFrame(PositionConstraint position_constraint);
    bool evaluateConstraint(const tf::Point& p);
    bool calculateMapConstraintArea(std::vector<unsigned int>& mx, std::vector<unsigned int>& my, std::vector<tf::Point>& goal_positions);
    void planToWorld(const std::vector<int>& plan_xs, const std::vector<int>& plan_ys, std::vector<geometry_msgs::PoseStamped>& plan);

    PositionConstraint position_constraint_;
    std::vector<tf::Point> goal_positions_in_constraint_frame_;
};

#endif

}
