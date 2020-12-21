cmake_minimum_required(VERSION 3.0.2)
project(ocpl_tsr)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  ocpl_sampling
)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS moveit_core moveit_ros_planning roscpp
  DEPENDS EIGEN3
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
  src/task_space_regions.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
