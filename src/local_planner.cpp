#include "cb_base_navigation/local_planner/local_planner_interface.h"

#include <costmap_2d/costmap_2d_ros.h>

#include <ros/init.h>
#include <ros/duration.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cb_local_planner_interface");

    // Transform listener
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    // Setup the global costmap
    costmap_2d::Costmap2DROS costmap("local_costmap", buffer);

    // Create the base controller interface
    cb_local_planner::LocalPlannerInterface lpi(&costmap, &buffer);

    ros::spin();

    return 0;
}
