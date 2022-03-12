# Constraint Based Base Navigation

[![CI](https://github.com/tue-robotics/cb_base_navigation/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/cb_base_navigation/actions/workflows/main.yml)

Alternative to move_base (navigation stack ROS); a more simplistic base bavigation which works with the Costmap 2D ROS world model. In can handle goal position constraints and the goal orientation constraint is set up in a different way (see cb_base_navigation_msgs_srvs package).

It provides two interfaces:

- Local Planner Interface: Interface to communicate with the local planner via actionlib. It works with local planners that adhere to the nav_core::base_local_planner interface.
- Global Planner Interface: Interface to communicate with the global planner, works with a new interface specified in cb_global_planner::GlobalPlannerPlugin. The following interaction is possible:
- Request a plan based on a set of position constraints
- Check if a plan is still valid

## Setup

Setup the following parameter files:

- params/local_planner.yaml
- params/local_costmap.yaml
- params/global_planner.yaml
- params/global_costmap.yaml

## Example

```bash
roslaunch cb_base_navigation local_planner.launch
```

```bash
roslaunch cb_base_navigation global_planner.launch
```

```bash
rosrun cb_base_navigation move '/person' 'x^2 + y^2 < r^2'
```
