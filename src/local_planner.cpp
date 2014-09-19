#include "cb_base_navigation/local_planner/local_planner_interface.h"

#include <tue/profiling/profiler.h>
#include <tue/profiling/ros/profile_publisher.h>
#include <tue/profiling/scoped_timer.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "cb_local_planner_interface");

    // Transform listener
    tf::TransformListener tf(ros::Duration(10));

    // Setup the global costmap
    costmap_2d::Costmap2DROS costmap("local_costmap", tf);

    // Create the base controller interface
    cb_local_planner::LocalPlannerInterface lpi(&costmap);

    ros::spin();

    return 0;
}
