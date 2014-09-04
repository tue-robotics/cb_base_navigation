#include "cb_base_navigation/local_planner/local_planner_interface.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "cb_local_planner_interface");

    // Transform listener
    tf::TransformListener tf(ros::Duration(10));

    // Setup the global costmap
    costmap_2d::Costmap2DROS costmap("local_costmap", tf);

    // Create the base controller
    cb_local_planner::LocalPlannerInterface lpi(costmap);

    // Frequency of the base controller
    ros::Rate r(5);

    bool costmap_thread_running = costmap.getMapUpdateFrequency() > 0;

    if (costmap_thread_running)
        ROS_INFO("LPI: Costmap thread is running: Lock required each cycle.");
    else
        ROS_INFO("LPI: Costmap thread not running: costmap will be updated in the loop.");

    while(ros::ok()) {

        // Check if we have a costmap thread running
        costmap_thread_running = costmap.getMapUpdateFrequency() > 0;

        // If we're not running an extra thread to update the costmap, do it in this loop
        if (costmap_thread_running)
        {
            // Prevent the costmap from updating
            boost::unique_lock< boost::shared_mutex > lock(*(costmap.getCostmap()->getLock()));

            // Control the base
            lpi.doSomeMotionPlanning();

            // Let the costmap continue updating
            lock.unlock();
        }
        else
        {
            // Update the costmap just before we're going to control the base
            costmap.updateMap();

            // Control the base
            lpi.doSomeMotionPlanning();

            // Publish the map for visualization
            costmap.getPublisher()->publishCostmap();
        }

        // Spin and sleep
        ros::spinOnce();
        r.sleep();
    }

    return(0);
}
