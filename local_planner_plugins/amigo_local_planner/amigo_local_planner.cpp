// Created by Sjoerd van den Dries, 2011

#include "amigo_local_planner.h"
#include <pluginlib/class_list_macros.h>


using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(amigo_local_planner, AmigoLocalPlanner, cb_local_planner::AmigoLocalPlanner, nav_core::BaseLocalPlanner)

namespace cb_local_planner {

    const double PI = 3.141592654;

    AmigoLocalPlanner::AmigoLocalPlanner() : initialized_(false), tf_(NULL)  {}


    void AmigoLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        //void AmigoLocalPlanner::initialize(const std::string& name, tf::TransformListener* tf, const std::vector<tue_costmap_2d::Costmap2DROS*>& ros_costmaps) {

        /*
        if (ros_costmaps.empty()) {
            ROS_WARN("Costmap list is empty!");
            return;
        }
        combined_costmap_.addCostmaps(ros_costmaps);
        combined_costmap_.advertise(private_nh);

        costmap_frame_ = combined_costmap_.getGlobalFrameID();
        robot_base_frame_ = combined_costmap_.getBaseFrameID();
         */

        costmap_ros_ = costmap_ros;

        //initialize the copy of the costmap the controller will use
        costmap_ = *costmap_ros_->getCostmap();

        costmap_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        tf_ = tf;

        ros::NodeHandle private_nh("~/" + name);

        global_frame_ = "/map";

        t_last_cmd_vel_ = 0;

        // parameters
        private_nh.param("max_vel_translation", MAX_VEL, 0.6);
        private_nh.param("max_acc_translation", MAX_ACC, 0.6);
        private_nh.param("max_vel_rotation", MAX_VEL_THETA, 0.6);
        private_nh.param("max_acc_rotation", MAX_ACC_THETA, 1.0);
        private_nh.param("xy_goal_tolerance", XY_GOALREGION, 0.1);
        private_nh.param("yaw_goal_tolerance", THETA_GOALREGION, 0.1);

        private_nh.param("max_yaw_error_driving", MAX_YAW_ERROR_DRIVING, 0.785398164); // 0.25 * PI
        private_nh.param("max_yaw_error_still", MAX_YAW_ERROR_STILL, 0.1);
        private_nh.param("still_max_vel", STILL_MAX_VEL, 0.01);

        private_nh.param("c_theta/kp", c_theta.kp, 1.0);
        private_nh.param("c_theta/ki", c_theta.ki, 0.0);
        private_nh.param("c_theta/kd", c_theta.kd, 0.0);

        double max_path_distance;
        private_nh.param("max_strafe_distance", MAX_STRAFE_DISTANCE, 0.4);
        private_nh.param("max_path_distance", max_path_distance, 0.5);
        MAX_PATH_DISTANCE_SQ = max_path_distance * max_path_distance;

        private_nh.param("max_local_goal_distance", MAX_LOCAL_GOAL_DISTANCE, 0.5);
        private_nh.param("look_ahead", LOOKAHEAD, 2.0);
        private_nh.param("gain", GAIN, 0.9);

        STILL_MAX_VEL_SQ = STILL_MAX_VEL * STILL_MAX_VEL;

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

        initialized_ = true;
    }


    AmigoLocalPlanner::~AmigoLocalPlanner() {
    }
    bool AmigoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan_msg) {
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //reset the global plan
        global_plan_.clear();
        global_plan_.resize(global_plan_msg.size());

        for(unsigned int i = 0; i < global_plan_.size(); ++i) {
            tf::poseStampedMsgToTF(global_plan_msg[i], global_plan_[i]);

            // the waypoints are timeless (as in: at any moment in time, we want to pursue this plan)
            // this is needed for the tf::transformPose to be determined based on the latest time
            // (instead of the time of the creation of the plan)
            global_plan_[i].stamp_ = ros::Time();
        }

        global_goal_GLOBAL_FRAME_ = global_plan_.back();

        return true;
    }

    bool AmigoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (global_plan_.empty()) {
            ROS_ERROR("Received plan is empty");
            return false;
        }

        // determine dt since last callback
        double time = ros::Time::now().toSec();
        double dt = 0;
        if (t_last_cmd_vel_ > 0) {
            dt = time - t_last_cmd_vel_;
        }
        t_last_cmd_vel_ = time;

        //make sure to update the costmap we'll use for this cycle
        //combined_costmap_.update();
        costmap_ = *costmap_ros_->getCostmap();

        // determine robot pose
        tf::Stamped<tf::Pose> robot_pose_COSTMAP_FRAME;
        //combined_costmap_.getRobotPose(robot_pose_COSTMAP_FRAME);
        costmap_ros_->getRobotPose(robot_pose_COSTMAP_FRAME);
        robot_pose_COSTMAP_FRAME.stamp_ = ros::Time();

        //combined_costmap_.publishCostmap(robot_pose_COSTMAP_FRAME.getOrigin().getX(), robot_pose_COSTMAP_FRAME.getOrigin().getY(), 100);

        tf::Stamped<tf::Pose> robot_pose_GLOBAL_FRAME;
        try {
            tf_->transformPose(global_frame_, robot_pose_COSTMAP_FRAME, robot_pose_GLOBAL_FRAME);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return false;
        }

        // determine the index of the waypoint on the global path closest to the robot
        double min_dist_sq = MAX_PATH_DISTANCE_SQ;
        int current_waypoint_index = -1;
        for(unsigned int i = 0; i < global_plan_.size(); ++i) {
            const tf::Stamped<tf::Pose>& waypoint = global_plan_[i];
            double dist_sq = (waypoint.getOrigin() - robot_pose_GLOBAL_FRAME.getOrigin()).length2();

            if (dist_sq < min_dist_sq) {
                current_waypoint_index = i;
                min_dist_sq = dist_sq;
            }
        }

        ROS_DEBUG_STREAM("current_waypoint = " << current_waypoint_index);

        if (current_waypoint_index < 0) {
            ROS_WARN("Global path is too far! Waiting for new global path...");
            return false;
        }

        bool is_clear_global_path = true;
        double waypoint_distance = 0;
        //tf::Stamped<tf::Pose>& last_waypoint = global_plan_[current_waypoint_index];
        tf::Stamped<tf::Pose> local_goal_COSTMAP_FRAME;

        // check if every waypoint on the global path up to the LOOKAHEAD distance is clear
        // UPDATE BAS: now the entire global path is first checked up to lookahead
        // next the path is checked for a clear line of sight up to MAX_LOCAL_GOAL_DISTANCE
        for(int i_waypoint = current_waypoint_index; waypoint_distance < LOOKAHEAD && i_waypoint < (int)global_plan_.size(); ++i_waypoint) {
            const tf::Stamped<tf::Pose>& waypoint = global_plan_[i_waypoint];
            tf::Stamped<tf::Pose> waypoint_COSTMAP_FRAME;

            try {
                // todo: calculate transform out of loop and use it here
                tf_->transformPose(costmap_frame_, waypoint, waypoint_COSTMAP_FRAME);
                waypoint_COSTMAP_FRAME.stamp_ = ros::Time();
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                setZeroVelocity(cmd_vel);
                return false;
            }

            unsigned int mx, my;
            costmap_.worldToMap(waypoint_COSTMAP_FRAME.getOrigin().getX(), waypoint_COSTMAP_FRAME.getOrigin().getY(), mx, my);
            unsigned char cost = costmap_.getCost(mx, my);

            if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION) {
                is_clear_global_path = false;
                break;
            }

            if (i_waypoint < (int)global_plan_.size() - 1) {
                waypoint_distance += (global_plan_[i_waypoint + 1].getOrigin() - global_plan_[i_waypoint].getOrigin()).length();
            }
        }

        if (!is_clear_global_path) {
            return false;
        }

        // a new subtarget can be determined if the global path is clear
        waypoint_distance = 0; // reset
        int local_goal_index = -1;
        // check if every waypoint up to the local goal has a clear line of sight
        // this acts as a carrot in front of the robot
        for(int i_waypoint = current_waypoint_index; waypoint_distance < MAX_LOCAL_GOAL_DISTANCE && i_waypoint < (int)global_plan_.size(); ++i_waypoint) {
            const tf::Stamped<tf::Pose>& waypoint = global_plan_[i_waypoint];
            tf::Stamped<tf::Pose> waypoint_COSTMAP_FRAME;

            try {
                // todo: calculate transform out of loop and use it here
                tf_->transformPose(costmap_frame_, waypoint, waypoint_COSTMAP_FRAME);
                waypoint_COSTMAP_FRAME.stamp_ = ros::Time();
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                setZeroVelocity(cmd_vel);
                return false;
            }

            // if there is not clear line between the robot and the local goal, the previous waypoint is the local goal
            if (!isClearLine(robot_pose_COSTMAP_FRAME.getOrigin(), waypoint_COSTMAP_FRAME.getOrigin())) {
                // BAS: neccessary to set global path to false here as well? if the global path is checked and the obstacle
                // is within MAX_LOCAL_GOAL_DISTANCE it will break but the false will never be returned, hence a velocity
                // will be returned to achieve the last waypoint before the obstacle
                // Theoretically the robot should drive up to the obstacle, i.e., the feasible waypoint must be set as the
                // local goal. however due to localization errors/sensor noise/overshoot the robot may hit the obstacle.
                // it is therefore safer to immediately stop, thus set is_clear_global_path to false.
                // TODO: implement a parameter that defines up to what the robot may continue to approach the obstacle
                // MAX_LOCAL_GOAL_DISTANCE can then still be used
                // it is better to check if the velocity is determined such that it is ensured that the robot has zero
                // velocity at the point it is supposed to stop. so instead of hardcoding a parameter that defines a safe
                // distance it is better to determine this based on the current velocity and the maximum decelleration.
                //is_clear_global_path = false; // temporary workaround to ensure a stop if an obstacle is encountered within MAX_LOCAL_GOAL_DISTANCE
                break;
            } else {
                local_goal_index = i_waypoint;
                local_goal_COSTMAP_FRAME = waypoint_COSTMAP_FRAME;
            }

            if (i_waypoint < (int)global_plan_.size() - 1) {
                waypoint_distance += (global_plan_[i_waypoint + 1].getOrigin() - global_plan_[i_waypoint].getOrigin()).length();
            }
        }

        // if there is no clear line of sight to one of the waypoints on the global path, stand still.
        // move_base will request a re-plan from the global planner
        if (local_goal_index < 0) {
            ROS_WARN("Cannot get to first waypoint");
            return false;
        }


        // determine distance to global goal along path
        double path_distance_to_global_goal = 0;
        for(int i_waypoint = current_waypoint_index; i_waypoint < (int)global_plan_.size() - 1; ++i_waypoint) {
            path_distance_to_global_goal += (global_plan_[i_waypoint + 1].getOrigin() - global_plan_[i_waypoint].getOrigin()).length();
        }

        // transform the goal to the base frame --> the error can therefore be defined as the local goal
        tf::Stamped<tf::Pose> local_goal_BASE_FRAME;
        try {
            tf_->transformPose(robot_base_frame_, local_goal_COSTMAP_FRAME, local_goal_BASE_FRAME);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return false;
        }

        // if the transform succeeds the cmd_vel can be determined
        tf::Vector3 e_pos = local_goal_BASE_FRAME.getOrigin();

        // if we want to strafe, the desired orientation of the robot is the orientation of the global goal
        double e_theta = 0;
        if (path_distance_to_global_goal < MAX_STRAFE_DISTANCE) {
            // transform the global goal to the base frame --> the error can therefore be defined as the local goal
            tf::Stamped<tf::Pose> global_goal_BASE_FRAME;
            try {
                tf_->transformPose(robot_base_frame_, global_goal_GLOBAL_FRAME_, global_goal_BASE_FRAME);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                return false;
            }

            e_theta = tf::getYaw(global_goal_BASE_FRAME.getRotation());
            // else it must turn towards local goal
        } else {
            e_theta = calculateHeading(tf::Vector3(0, 0, 0), e_pos);
        }

        geometry_msgs::Twist e_pos_norm_msg;
        tf::Vector3 e_pos_norm;
        e_pos_norm = e_pos.normalized();
        tf::vector3TFToMsg(e_pos_norm, e_pos_norm_msg.linear);

        ROS_DEBUG_STREAM("e_pos = " << e_pos_norm_msg.linear << ", e_theta = " << e_theta);
        ROS_DEBUG_STREAM("path_distance_to_global_goal = " << path_distance_to_global_goal << ", dt = " << dt);

        determineDesiredVelocity(e_pos, e_theta, last_cmd_vel_, path_distance_to_global_goal, dt, cmd_vel);

        ROS_DEBUG_STREAM("last_cmd_vel = " << last_cmd_vel_.linear << ", cmd_vel = " << cmd_vel.linear);

        last_cmd_vel_ = cmd_vel;

        // publish plan
        publishPlan(local_goal_BASE_FRAME, plan_pub_);

        return true;
    }

    bool AmigoLocalPlanner::isGoalReached(){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (global_plan_.empty()) {
            // no goal, so by definition, goal is reached
            return true;
        }

        // transform the goal to the base frame
        tf::Stamped<tf::Pose> global_goal_BASE_FRAME;
        try {
            tf_->transformPose(robot_base_frame_, global_goal_GLOBAL_FRAME_, global_goal_BASE_FRAME);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return false;
        }

        // check if robot is close enough to goal position
        if (global_goal_BASE_FRAME.getOrigin().length() > XY_GOALREGION) {
            // too far from goal
            return false;
        }

        if (last_cmd_vel_.linear.x * last_cmd_vel_.linear.x + last_cmd_vel_.linear.y * last_cmd_vel_.linear.y > STILL_MAX_VEL_SQ) {
            // still driving
            return false;
        }

        // check if robot is close enough to goal orientation
        double e_theta = tf::getYaw(global_goal_BASE_FRAME.getRotation());
        if (fabs(e_theta) > THETA_GOALREGION) {
            // not oriented right
            return false;
        }

        // robot reached the goal
        global_plan_.clear();

        return true;
    }


    bool AmigoLocalPlanner::isClearLine(const tf::Vector3& p1, const tf::Vector3& p2) {
        unsigned int mx1, mx2, my1, my2;

        if (!costmap_.worldToMap(p1.getX(), p1.getY(), mx1, my1)) return false;

        if (!costmap_.worldToMap(p2.getX(), p2.getY(), mx2, my2)) return false;

        if (mx1 == mx2 && my1 == my2) {
            return true;
        }

        /// Compute the number of steps in x and y direction
        int imxdiff = mx2 - mx1;
        int imydiff = my2 - my1;

        /// The number of steps to take is the maximum of both
        unsigned int n_steps = max(fabs(imxdiff), fabs(imydiff));

        double dmxdiff = (p2.getX() - p1.getX())/n_steps;
        double dmydiff = (p2.getY() - p1.getY())/n_steps;

        for (unsigned int step = 0; step <= n_steps; step++) {

            /// Compute exact point on line
            double px = p1.getX() + step * dmxdiff;
            double py = p1.getY() + step * dmydiff;

            /// Convert to cell
            unsigned int mx, my;
            if (!costmap_.worldToMap(px, py, mx, my)) {
                ROS_INFO("No clear line due to failed transform from world to map");
                return false;
            }

            /// Check collision
            double cost = costmap_.getCost(mx, my);

            if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION) {
                //ROS_WARN_STREAM("Line cost = " << cost << " at [mx my] = [" << mx << " " << my << "]");
                ROS_WARN_STREAM("Line cost = " << cost << " at [x y] = [" << px << " " << py << "]");
                return false;
            }
        }
/*
        double mxdiff = (double)mx2 - (double)mx1;
        double mydiff = (double)my2 - (double)my1;

        double mdx = 0;
        double mdy = 0;
        if (fabs(mxdiff) < fabs(mydiff)) {
            mdx = mxdiff / fabs(mydiff);
            mdy = sign(mydiff);
        } else {
            mdx = sign(mxdiff);
            mdy = mydiff / fabs(mxdiff);
        }

        double mx = mx1;
        double my = my1;

        while((unsigned int)mx != mx2 && (unsigned int)my != my2) {
            unsigned char cost = costmap_.getCost((int)mx, (int)my);
            if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION) {
                return false;
            }

            mx += mdx;
            my += mdy;
        }

        unsigned char cost = costmap_.getCost((int)mx, (int)my);
        if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION) {
            return false;
        }
*/
        return true;
    }

    double AmigoLocalPlanner::calculateHeading(const tf::Vector3& p1, const tf::Vector3& p2) {
        return atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
    }

    void AmigoLocalPlanner::determineDesiredVelocity(tf::Vector3 e_pos, double e_theta,
                                                     const geometry_msgs::Twist& current_vel, double path_distance_to_global_goal, double dt, geometry_msgs::Twist& cmd_vel) {

        double e_norm = e_pos.length();

        tf::Vector3 current_vel_trans;
        tf::vector3MsgToTF(current_vel.linear, current_vel_trans);
        double v_norm_sq = current_vel_trans.length2();

        // determine the maximum error of the orientation,
        // if this error is reached the robot stops and only controls the rotation

        // during execution of the path
        double max_theta_error = MAX_YAW_ERROR_DRIVING;

        if (v_norm_sq < STILL_MAX_VEL_SQ) {
            // when the robot is standing still
            max_theta_error = MAX_YAW_ERROR_STILL;
        }

        double v_wanted_norm = 0;

        // if the rotation error is smaller than the max rotation error and the local goal is not reached yet
        // or the local goal is the global goal --> control translation
        if ((fabs(e_theta) < max_theta_error && e_norm > 0) || path_distance_to_global_goal < MAX_STRAFE_DISTANCE) {
            // DEPRECATED
            //v_wanted_norm = min(MAX_VEL, GAIN * sqrt(2 * e_norm * MAX_ACC));
            //v_wanted_norm = min(MAX_VEL, GAIN * sqrt(2 * path_distance_to_global_goal * MAX_ACC));

            // NEW
            // - path_distance_to_global_goal defines the distance from the closest waypoint (to the robot)
            //   to the global goal along the path
            // - e_norm defines the error between the robot's location and the local goal,
            //   defined by the number of waypoints that is clear up to MAX_LOCAL_GOAL_DISTANCE.
            // if e_norm is defined to calculate the wanted velocity it will cause a slow speed if the number of free waypoints
            // up to MAX_LOCAL_GOAL_DISTANCE is small; this happens when making sharp turns around an obstacle
            // if path_distance_to_global_goal is chosen the robot will maintain velocity in such cases as the global goal is used
            // to calculate the wanted velocity. however, if the robot is near the goal path_distance_to_global_goal will be 0
            // (as the closest waypoint to the robot is the goal!) and thus the wanted velocity will be set to 0. this can cause
            // the robot to never reach the goal as there still might be an error in the local frame (e_norm != 0), due to, e.g.,
            // overshoot
            //
            // solution: take the maximum of both errors
            // if the local error is greater, this means the robot is close to the goal but a velocity is still neccesary to reach the goal
            // if the global error is greater, the robot will maintain a nice pace towards the goal
            double error_pos = max(e_norm, path_distance_to_global_goal);

            v_wanted_norm = min(MAX_VEL, GAIN * sqrt(2 * error_pos * MAX_ACC));
        }

        // make sure the wanted velocity has the direction towards the goal and the magnitude of v_wanted_norm
        tf::Vector3 vel_wanted;
        vel_wanted = e_pos.normalized();
        vel_wanted *= v_wanted_norm;

        // check if the acceleration bound is violated
        tf::Vector3 vel_diff = vel_wanted - current_vel_trans;
        double acc_wanted = vel_diff.length() / dt;
        if (acc_wanted > MAX_ACC) {
            tf::vector3TFToMsg(current_vel_trans + vel_diff.normalized() * MAX_ACC * dt, cmd_vel.linear);
        } else {
            tf::vector3TFToMsg(vel_wanted, cmd_vel.linear);
        }

        // the rotation is always controlled
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        //cmd_vel.angular.z = determineReference(e_theta, current_vel.angular.z, MAX_VEL_THETA, MAX_ACC_THETA, dt);
        cmd_vel.angular.z = updateRotationControl(e_theta, current_vel.angular.z, dt);
    }

    double AmigoLocalPlanner::updateRotationControl(const double &error, const double &current_velocity, const double &dt) {

        /// Apply proportional action
        double output = c_theta.kp * error;

        /// Apply derivative action: assume vref = 0.0
        output -= c_theta.kd * current_velocity;

        // ToDo: apply integral action

        /// Limit velocity
        output = max(-MAX_VEL_THETA, min(MAX_VEL_THETA, output));

        /// Limit acceleration
        double acc = (output - current_velocity)/dt;
        if (acc > MAX_ACC_THETA) {
            output = current_velocity + MAX_ACC_THETA * dt;
        } else if (acc < -MAX_ACC_THETA) {
            output = current_velocity - MAX_ACC_THETA * dt;
        }
        return output;

    }

    // PARTLY TAKEN FROM amigo_ref_interpolator
    double AmigoLocalPlanner::determineReference(double error_x, double vel, double max_vel, double max_acc, double dt) {
        double EPS = 0.5 * max_acc*dt;

        //initial state
        bool still = false;
        bool move = false;
        bool dec = false;
        bool con = false;
        bool acc = false;

        double a = 0.0;
        double vel_mag = fabs(vel);

        //compute deceleration distance
        double delta_t1=vel_mag/max_acc; //deceleration segment time
        double dec_dist = 0.5*max_acc * (delta_t1) * (delta_t1); //deceleration distance

        //determine magnitude and sign of error vector
        double delta_x = fabs(error_x);
        int sign_x = sign(vel);

        //decide whether to move or stand still
        if (vel_mag!=0.0 || delta_x > EPS){
            move = true;
        } else {
            still = true;
            error_x = 0;
        }
        double dir = sign(error_x);

        //move: decide whether to stop, decelerate, constant speed or accelerate
        if (move){
            //		if (stopping){
            //			acc = false;
            //			con = false;
            //			still = false;
            //			dec = true;
            //			///ROS_ERROR("stopping");
            //       	} else
            if (fabs(dec_dist) >= fabs(delta_x)){
                dec = true;
                // ROS_INFO("go to dec");
            }
            else if (sign_x * error_x < 0 && vel_mag != 0.0){
                dec = true;
                // ROS_INFO("setpoint behind");
            }
            else if (fabs(dec_dist) < fabs(delta_x) && vel_mag >= max_vel){
                con = true;
                // ROS_INFO("go to con");
            }
            else {
                acc = true;
                // ROS_INFO("go to acc");
            }

            //move: reference value computations
            if (acc){
                vel_mag += max_acc * dt;
                vel_mag = std::min<double>(vel_mag, max_vel);
                //x+= dir * vel_mag * dt;
                a = dir * max_acc;
            }
            if (con){
                //x+= dir * vel_mag * dt;
                a = 0;
            }
            if (dec){
                vel_mag -= max_acc * dt;
                vel_mag = std::max<double>(vel_mag, 0.0);
                //x+= dir * vel_mag * dt;
                a = - dir * max_acc;
                if (vel_mag < (0.5 * max_acc * dt)){
                    vel_mag = 0.0;
                    //reset = true;
                    ///ROS_WARN("reset");
                }

            }

            //ready = false;

        }

        //stand still: reset values
        else if (still){
            vel = 0;
            a = 0;
            sign_x = 0;
            //reset = true;
            //ready = true;

        }
        else {
        }

        vel = dir * vel_mag;
        return vel;
    }

    void setZeroVelocity(geometry_msgs::Twist& cmd_vel) {
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;

        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
    }

    void AmigoLocalPlanner::publishPlan(const tf::Stamped<tf::Pose>& local_goal, ros::Publisher& pub) {
        if (local_goal.frame_id_ != robot_base_frame_) {
            return;
        }

        //create a path message
        nav_msgs::Path gui_path;
        gui_path.header.frame_id = robot_base_frame_;
        gui_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped zero_pose_msg_BASE_FRAME;
        zero_pose_msg_BASE_FRAME.header.frame_id = robot_base_frame_;
        zero_pose_msg_BASE_FRAME.header.stamp = ros::Time::now();
        zero_pose_msg_BASE_FRAME.pose.position.x = 0;
        zero_pose_msg_BASE_FRAME.pose.position.y = 0;
        zero_pose_msg_BASE_FRAME.pose.position.z = 0;
        zero_pose_msg_BASE_FRAME.pose.orientation.x = 0;
        zero_pose_msg_BASE_FRAME.pose.orientation.y = 0;
        zero_pose_msg_BASE_FRAME.pose.orientation.z = 0;
        zero_pose_msg_BASE_FRAME.pose.orientation.w = 1;

        geometry_msgs::PoseStamped local_goal_msg_BASE_FRAME;
        tf::poseStampedTFToMsg(local_goal, local_goal_msg_BASE_FRAME);

        gui_path.poses.push_back(zero_pose_msg_BASE_FRAME);
        gui_path.poses.push_back(local_goal_msg_BASE_FRAME);

        pub.publish(gui_path);
    }

    /*
    void AmigoLocalPlanner::printPose(const std::string& text, const tf::Stamped<tf::Pose>& p) const {
        cout << text << " (" << p.frame_id_ << "): " << p.getOrigin().getX() << ", " << p.getOrigin().getY() << ", " << p.getOrigin().getZ()  << endl;
    }
     */

};
