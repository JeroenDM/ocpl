cmake_minimum_required(VERSION 3.0.2)
project(ocpl_graph)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS moveit_core moveit_ros_planning roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
)

add_library(
  ${PROJECT_NAME}
  src/tree.cpp
  src/graph.cpp
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)

#   catkin_add_gtest(${PROJECT_NAME}_test test/test_sampling.cpp)

#   target_link_libraries(${PROJECT_NAME}_test
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )

# endif()
