cmake_minimum_required(VERSION 2.8.3)
project(cb_base_navigation)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  base_local_planner
  cb_base_navigation_msgs
  code_profiler
  costmap_2d
  ed_msgs
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  tf2
  tf2_ros
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_planners
  CATKIN_DEPENDS actionlib base_local_planner cb_base_navigation_msgs code_profiler costmap_2d ed_msgs geometry_msgs nav_core pluginlib roscpp tf2 tf2_ros visualization_msgs
)

###########
## Build ##
###########

add_compile_options(-Wall)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  3rdparty
  ${catkin_INCLUDE_DIRS}
)

# Declare libraries
file(GLOB_RECURSE GLOBAL_PLANNER_SRC_FILES global_planner_plugins/*.cpp)
add_library(global_planners ${GLOBAL_PLANNER_SRC_FILES})
target_link_libraries(global_planners ${catkin_LIBRARIES})

# get all the header files (for qtcreator)!
file(GLOB_RECURSE LOCAL_HEADERS include/cb_base_navigation/local_planner/*.h)
file(GLOB_RECURSE GLOBAL_HEADERS include/cb_base_navigation/global_planner/*.h)

# get all the src files !
file(GLOB_RECURSE LOCAL_PLANNER_INTERFACE_SOURCES src/local_planner/*.cpp)
file(GLOB_RECURSE GLOBAL_PLANNER_INTERFACE_SOURCES src/global_planner/*.cpp)

## Local planner node
add_executable(local_planner_interface src/local_planner.cpp ${LOCAL_PLANNER_INTERFACE_SOURCES} ${LOCAL_HEADERS})
target_link_libraries(local_planner_interface ${catkin_LIBRARIES})
add_dependencies(local_planner_interface ${catkin_EXPORTED_TARGETS})

## Global planner node
add_executable(global_planner_interface src/global_planner.cpp ${GLOBAL_PLANNER_INTERFACE_SOURCES} ${GLOBAL_HEADERS})
target_link_libraries(global_planner_interface ${catkin_LIBRARIES})
add_dependencies(global_planner_interface ${catkin_EXPORTED_TARGETS})
