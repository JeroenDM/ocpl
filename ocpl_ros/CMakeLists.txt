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
  moveit_msgs
  rospy
  tf2
  eigen_conversions
  roscpp
  ocpl_sampling
  ocpl_graph
  ocpl_tsr
  ocpl_planning
  ocpl_benchmark
  simple_moveit_wrapper
  trajectory_msgs
  moveit_ros_planning_interface
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
  src/rviz.cpp
  src/io.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_planning_server
  src/planning_server.cpp
)
set_target_properties(${PROJECT_NAME}_planning_server
  PROPERTIES OUTPUT_NAME planning_server PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_planning_server
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_show_task
  src/show_task.cpp
)
set_target_properties(${PROJECT_NAME}_show_task
  PROPERTIES OUTPUT_NAME show_task PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_show_task
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# add_executable(${PROJECT_NAME}_demo
#   demos/demo.cpp
# )
# set_target_properties(${PROJECT_NAME}_demo
#   PROPERTIES OUTPUT_NAME demo PREFIX ""
# )
# target_link_libraries(${PROJECT_NAME}_demo
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
# )

add_executable(${PROJECT_NAME}_oriolo
  demos/oriolo.cpp
)
set_target_properties(${PROJECT_NAME}_oriolo
  PROPERTIES OUTPUT_NAME oriolo PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_oriolo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_play_path
  demos/play_path.cpp
)
set_target_properties(${PROJECT_NAME}_play_path
  PROPERTIES OUTPUT_NAME play PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_play_path
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_demo_redundant
  demos/demo_redundant.cpp
)
set_target_properties(${PROJECT_NAME}_demo_redundant
  PROPERTIES OUTPUT_NAME demo_redundant PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_demo_redundant
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# welding demo with kuka kr5
add_executable(${PROJECT_NAME}_kuka
  demos/kuka.cpp
)
set_target_properties(${PROJECT_NAME}_kuka
  PROPERTIES OUTPUT_NAME kuka PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_kuka
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# welding demo with kuka kr5 on rail
add_executable(${PROJECT_NAME}_kuka_on_rail
  demos/kuka_on_rail.cpp
)
set_target_properties(${PROJECT_NAME}_kuka_on_rail
  PROPERTIES OUTPUT_NAME kuka_on_rail PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_kuka_on_rail
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# welding demo with kuka kr5 on rail
add_executable(${PROJECT_NAME}_rotation_table
  demos/rotation_table.cpp
)
set_target_properties(${PROJECT_NAME}_rotation_table
  PROPERTIES OUTPUT_NAME rotation_table PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_rotation_table
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


add_executable(${PROJECT_NAME}_benchmark1
  benchmarks/benchmark1.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark1
  PROPERTIES OUTPUT_NAME benchmark1 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark1
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_benchmark2
  benchmarks/benchmark2.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark2
  PROPERTIES OUTPUT_NAME benchmark2 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark2
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_benchmark3
  benchmarks/benchmark3.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark3
  PROPERTIES OUTPUT_NAME benchmark3 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark3
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_benchmark4
  benchmarks/benchmark4.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark4
  PROPERTIES OUTPUT_NAME benchmark4 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark4
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_benchmark5
  benchmarks/benchmark5.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark5
  PROPERTIES OUTPUT_NAME benchmark5 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark5
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

############# 
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(${PROJECT_NAME}_test_io test/test_io.cpp)

  target_link_libraries(${PROJECT_NAME}_test_io
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

endif()
