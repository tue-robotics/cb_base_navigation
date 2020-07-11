/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_ASTARPLANNER_GPP_H_
#define cb_global_planner_ASTARPLANNER_GPP_H_

#include "a_star_planner.h"
#include "cb_base_navigation/global_planner/global_planner_plugin.h"
#include "cb_base_navigation/global_planner/constraint_evaluator.h"

#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <ros/service_client.h>

#include <tf2/transform_datatypes.h>

#include <std_msgs/ColorRGBA.h>

#include <visualization_msgs/Marker.h>

#include <memory>

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
     * @param  tf Pointer to the tf2_ros::Buffer
	 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
	 */
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap_ros);

    /**
     * @brief Given a set of goal constraints, compute a plan
     * @param start The start pose
     * @param pc Position contraints on the goal position
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start, const cb_base_navigation_msgs::PositionConstraint& pc,
                  std::vector<geometry_msgs::PoseStamped>& plan, std::vector<tf2::Vector3>& goal_positions);

    /**
     * @brief Check if plan is valid
     * @param plan plan to check
     * @return Validity of the plan
     */
    bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

private:

    costmap_2d::Costmap2DROS* global_costmap_ros_;
    std::unique_ptr<AStarPlanner> planner_;
    tf2_ros::Buffer* tf_;
    bool initialized_;

    bool constraintChanged(const cb_base_navigation_msgs::PositionConstraint& pc) { return (pc_.constraint != pc.constraint || pc_.frame != pc.frame); }

    bool updateConstraintPositionsInConstraintFrame(const cb_base_navigation_msgs::PositionConstraint& pc);
    bool evaluateConstraint(const tf2::Vector3& p);
    bool calculateMapConstraintArea(std::vector<unsigned int>& mx, std::vector<unsigned int>& my, std::vector<tf2::Vector3>& goal_positions);
    void planToWorld(const std::vector<int>& plan_xs, const std::vector<int>& plan_ys, std::vector<geometry_msgs::PoseStamped>& plan);

    cb_base_navigation_msgs::PositionConstraint pc_;
    std::vector<tf2::Vector3> goal_positions_in_constraint_frame_;

    //! World model client
    ros::ServiceClient ed_client_;

    bool queryEntityPose(const std::string& id, tf2::Transform& pose);

};

#endif

}
