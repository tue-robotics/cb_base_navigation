// Created by Sjoerd van den Dries, 2011

#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_core/base_local_planner.h>

#include "nav_msgs/Path.h"

#include <string>

namespace cb_local_planner {

/**
   * @class TrajectoryPlannerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class AmigoLocalPlanner : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Default constructor for the ros wrapper
       */
      AmigoLocalPlanner();

      //void initialize(const std::string& name, tf::TransformListener* tf, const std::vector<tue_costmap_2d::Costmap2DROS*>& ros_costmaps);
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~AmigoLocalPlanner();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    protected:

      void setZeroVelocity(geometry_msgs::Twist& cmd_vel);

      double sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
      }

      void determineDesiredVelocity(tf::Vector3 e_pos, double e_theta,
              const geometry_msgs::Twist& current_vel, double path_distance_to_global_goal, double dt, geometry_msgs::Twist& cmd_vel);

      double determineReference(double error_x, double vel, double max_vel, double max_acc, double dt);

      // ToDo: make generic such that it can be used for other directions as well?
      /**
       * @brief updateRotationControl Computes desired angular velocity using a P(ID) controller,
       * keeping track of the maximum velocity and maximum acceleration.
       * @param error Current angular error
       * @param current_velocity
       * @param dt
       * @return desired angular velocity
       */
      double updateRotationControl(const double& error, const double& current_velocity, const double& dt);

      struct controlParameters {
          double kp, ki, kd;
      } c_theta;

      bool isClearLine(const tf::Vector3& p1, const tf::Vector3& p2);

      double calculateHeading(const tf::Vector3& p1, const tf::Vector3& p2);

      void publishPlan(const tf::Stamped<tf::Pose>& local_goal, ros::Publisher& pub);

      //void printPose(const std::string& text, const tf::Stamped<tf::Pose>& p) const;

      double MAX_VEL;
      double MAX_ACC;
      double MAX_VEL_THETA;
      double MAX_ACC_THETA;
      double MAX_LOCAL_GOAL_DISTANCE;
      double LOOKAHEAD;
      double XY_GOALREGION;
      double THETA_GOALREGION;
      double MAX_STRAFE_DISTANCE;

      double MAX_YAW_ERROR_DRIVING;
      double MAX_YAW_ERROR_STILL;
      double STILL_MAX_VEL;
      double STILL_MAX_VEL_SQ;

      double MAX_PATH_DISTANCE_SQ;

      double GAIN;

      bool initialized_;

      costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use

      costmap_2d::Costmap2D costmap_; ///< @brief The costmap the controller will use

      //tue_costmap_2d::CombinedCostmap combined_costmap_;

      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      std::string global_frame_; ///< @brief The frame in which the controller will run

      std::string costmap_frame_; ///< @brief The frame in which the controller will run

      std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot

      std::vector<tf::Stamped<tf::Pose> > global_plan_;

      tf::Stamped<tf::Pose> global_goal_GLOBAL_FRAME_;

      // index of the local goal in the global plan
      //int local_goal_index_;

      // timestamp of last time cmd_vel was published
      double t_last_cmd_vel_;

      geometry_msgs::Twist last_cmd_vel_;

      /*
      ros::Publisher pub_goal_;

      ros::Publisher pub_marker_;

      ros::Publisher pub_costmap_;

      ros::Publisher g_plan_pub_;
      */

      ros::Publisher plan_pub_;

      // Subscribe to the object detector
      ros::Subscriber sub_odom_;

      visualization_msgs::Marker checked_cells_marker_;

      int marker_id_;

  };

};

#endif
