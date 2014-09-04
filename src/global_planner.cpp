#include "cb_base_navigation/global_planner/global_planner_interface.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "cb_global_planner_interface");

    // Transform listener
    tf::TransformListener tf(ros::Duration(10));

    // Setup the global costmap
    costmap_2d::Costmap2DROS costmap("global_costmap", tf);
    costmap.start();

    // Create planner interface
    cb_global_planner::GlobalPlannerInterface gpi(costmap);

    // Spin :)
    ros::spin();

    return(0);
}
