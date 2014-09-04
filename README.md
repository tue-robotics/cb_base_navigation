cb_base_navigation
==================
Alternative to move_base (navigation stack ROS); a more simplistic base bavigation which works with the Costmap 2D ROS world model. In can handle goal position constraints and the goal orientation constraint is set up in a different way (see cb_base_navigation_msgs_srvs package).

It provides two interfaces:
- Local Planner Interface: Interface to communicate with the local planner via actionlib. It works with local planners that adhere to the nav_core::base_local_planner interface.
- Global Planner Interface: Interface to communicate with the global planner, works with a new interface specified in cb_global_planner::GlobalPlannerPlugin. The following interaction is possible:
 - Request a plan based on a set of position constraints
 - Check if a plan is still valid
