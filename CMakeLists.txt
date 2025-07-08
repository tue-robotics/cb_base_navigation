cmake_minimum_required(VERSION 3.5)
project(cb_base_navigation)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  base_local_planner
  ${PROJECT_NAME}_msgs
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
  LIBRARIES constraint_evaluator local_planner_interface global_planner_interface global_planner_plugins
  CATKIN_DEPENDS actionlib base_local_planner ${PROJECT_NAME}_msgs code_profiler costmap_2d ed_msgs geometry_msgs nav_core pluginlib roscpp tf2 tf2_ros visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  SYSTEM
  3rdparty
  ${catkin_INCLUDE_DIRS}
)

# Declare libraries
add_library(constraint_evaluator src/global_planner/constraint_evaluator.cpp)
target_link_libraries(constraint_evaluator ${catkin_LIBRARIES})

add_library(local_planner_interface
  src/local_planner/local_planner_interface.cpp
  src/local_planner/visualization.cpp
)
target_link_libraries(local_planner_interface ${catkin_LIBRARIES})
add_dependencies(local_planner_interface ${catkin_EXPORTED_TARGETS})

add_library(global_planner_interface
  src/global_planner/global_planner_interface.cpp
  src/global_planner/visualization.cpp
)
target_link_libraries(global_planner_interface ${catkin_LIBRARIES})
add_dependencies(global_planner_interface ${catkin_EXPORTED_TARGETS})

## Local planner node
add_executable(local_planner src/local_planner.cpp )
target_link_libraries(local_planner local_planner_interface ${catkin_LIBRARIES})

## Global planner node
add_executable(global_planner src/global_planner.cpp)
target_link_libraries(global_planner global_planner_interface ${catkin_LIBRARIES})

## Global planner plugins
file(GLOB_RECURSE GLOBAL_PLANNER_SRC_FILES global_planner_plugins/*.cpp)
add_library(global_planner_plugins ${GLOBAL_PLANNER_SRC_FILES})
target_link_libraries(global_planner_plugins constraint_evaluator global_planner_interface ${catkin_LIBRARIES})
add_dependencies(global_planner_plugins ${catkin_EXPORTED_TARGETS})

catkin_install_python(
  PROGRAMS examples/move
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS constraint_evaluator global_planner_interface global_planner_plugins local_planner_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    global_planner
    local_planner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES global_planner_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test(-W2)
endif()
