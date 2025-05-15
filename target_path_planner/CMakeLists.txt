cmake_minimum_required(VERSION 3.0.2)
project(target_path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -O3 -g -Wall)
add_compile_options(-std=c++17 -O3 -g -Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
nav_msgs
geometry_msgs
tf
tf2
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES target_path_planner
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME} src/target_path_planner.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/target_path_planner_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})