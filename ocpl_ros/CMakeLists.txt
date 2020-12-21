cmake_minimum_required(VERSION 3.0.2)
project(ocpl_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  roscpp
  ocpl_sampling
  ocpl_graph
  ocpl_tsr
  ocpl_planning
)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocpl_ros
#  CATKIN_DEPENDS moveit_core moveit_ros_planning moveit_visual_tools roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/moveit_robot.cpp
  src/moveit_robot_examples.cpp
  src/rviz.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_demo
  src/demo.cpp
)
set_target_properties(${PROJECT_NAME}_demo
  PROPERTIES OUTPUT_NAME demo PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_demo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
