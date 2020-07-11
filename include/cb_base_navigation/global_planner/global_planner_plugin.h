/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_GLOBAL_PLANNER_PLUGIN_
#define cb_global_planner_GLOBAL_PLANNER_PLUGIN_

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cb_base_navigation_msgs/PositionConstraint.h>

using namespace cb_base_navigation_msgs;

namespace cb_global_planner {
  /**
   * @class GlobalPlannerPlugin
   * @brief Provides an interface for global planners used in navigation.
   */
  class GlobalPlannerPlugin {
    public:
      /**
       * @brief Given a set of goal constraints, compute a plan
       * @param start The start pose 
       * @param position_constraint Position contraints on the goal position
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const tf::Stamped<tf::Pose>& start, const PositionConstraint& position_constraint, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<tf::Point>& goal_positions) = 0;

      /**
       * @brief Checks if a plan is valid
       * @param plan The plan that needs to be checked
       * @return True if a valid plan was valid, false otherwise
       */
      virtual bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief  Initialization function for the BaseGlobalPlanner
       * @param  name The name of this planner
       * @param  tf tf_listener
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~GlobalPlannerPlugin() {}

    protected:
      GlobalPlannerPlugin() {}
  };
}

#endif
