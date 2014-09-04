#include "cb_base_navigation/local_planner/local_planner_interface.h"

#include <tue/profiling/profiler.h>
#include <tue/profiling/ros/profile_publisher.h>
#include <tue/profiling/scoped_timer.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "cb_local_planner_interface");

    // Transform listener
    tf::TransformListener tf(ros::Duration(10));

    // Setup the global costmap
    costmap_2d::Costmap2DROS costmap("local_costmap", tf);

    // Create the base controller
    cb_local_planner::LocalPlannerInterface lpi(costmap);

    // Frequency of the base controller
    ros::Rate r(10);

    // Profiler
    tue::Profiler profiler;
    tue::ProfilePublisher profile_pub;
    profile_pub.initialize(profiler);

    bool costmap_thread_running = costmap.getMapUpdateFrequency() > 0;

    if (costmap_thread_running)
        ROS_INFO("LPI: Costmap thread is running: Lock required each cycle.");
    else
        ROS_INFO("LPI: Costmap thread not running: costmap will be updated in the loop.");

    while(ros::ok()) {

        tue::ScopedTimer timer(profiler, "total");

        // Check if we have a costmap thread running
        costmap_thread_running = costmap.getMapUpdateFrequency() > 0;

        // If we're not running an extra thread to update the costmap, do it in this loop
        if (costmap_thread_running)
        {
            // Prevent the costmap from updating
            boost::unique_lock< boost::shared_mutex > lock(*(costmap.getCostmap()->getLock()));

            // Control the base
            {
                tue::ScopedTimer timer(profiler, "LocalPlanner");
                lpi.doSomeMotionPlanning();
            }

            // Let the costmap continue updating
            lock.unlock();
        }
        else
        {
            // Update the costmap just before we're going to control the base
            {
                tue::ScopedTimer timer(profiler, "updateCostmap");
                costmap.updateMap();
            }

            // Control the base
            {
                tue::ScopedTimer timer(profiler, "LocalPlanner");
                lpi.doSomeMotionPlanning();
            }

            // Publish the map for visualization
            costmap.getPublisher()->publishCostmap();
        }

        // Publish feedback
        profile_pub.publish();

        // Spin and sleep
        ros::spinOnce();
        r.sleep();
    }

    return(0);
}
