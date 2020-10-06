/*******************************
 *                             *
 *  Author: Rein Appeldoorn    *
 *  Date:   2013-03-06         *
 *                             *
 *******************************/

#ifndef cb_global_planner_GLOBAL_PLANNER_PLUGIN_
#define cb_global_planner_GLOBAL_PLANNER_PLUGIN_

#include <cb_base_navigation_msgs/PositionConstraint.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2/utils.h>

#include <vector>

namespace costmap_2d {
class Costmap2DROS;
}

namespace tf2_ros {
class Buffer;
}

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
       * @param pc Position contraints on the goal position
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, const cb_base_navigation_msgs::PositionConstraint& pc,
                            std::vector<geometry_msgs::PoseStamped>& plan, std::vector<tf2::Vector3>& goal_positions) = 0;

      /**
       * @brief Checks if a plan is valid
       * @param plan The plan that needs to be checked
       * @return True if a valid plan was valid, false otherwise
       */
      virtual bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief  Initialization function for the BaseGlobalPlanner
       * @param  name The name of this planner
       * @param  tf tf buffer
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~GlobalPlannerPlugin() {}

    protected:
      GlobalPlannerPlugin() {}
  };
}

#endif
