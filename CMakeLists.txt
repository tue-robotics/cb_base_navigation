cmake_minimum_required(VERSION 2.8.3)
project(cb_base_navigation)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cb_planner_msgs_srvs
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  tf
  visualization_msgs
  actionlib
  actionlib_msgs
  base_local_planner
  code_profiler
  ed_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare libraries
file(GLOB_RECURSE GLOBAL_PLANNER_SRC_FILES global_planner_plugins/*.cpp)
add_library(global_planners ${GLOBAL_PLANNER_SRC_FILES})
target_link_libraries(global_planners ${catkin_LIBRARIES})

# get all the header files (for qtcreator)!
file(GLOB_RECURSE HEADERS include/*.h)

# get all the src files !
file(GLOB_RECURSE LOCAL_PLANNER_INTERFACE_SRC_FILES src/local_planner/*.cpp)
file(GLOB_RECURSE GLOBAL_PLANNER_INTERFACE_SRC_FILES src/global_planner/*.cpp)

## Declare a cpp executable
add_executable(cb_base_navigation_local_planner_interface src/local_planner.cpp ${LOCAL_PLANNER_INTERFACE_SRC_FILES} ${HEADERS})
add_executable(cb_base_navigation_global_planner_interface src/global_planner.cpp ${GLOBAL_PLANNER_INTERFACE_SRC_FILES} ${HEADERS})

target_link_libraries(cb_base_navigation_local_planner_interface ${catkin_LIBRARIES})
target_link_libraries(cb_base_navigation_global_planner_interface ${catkin_LIBRARIES})
