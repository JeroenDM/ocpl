cmake_minimum_required(VERSION 3.0.2)
project(ocpl_planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  ocpl_graph
  ocpl_sampling
  ocpl_tsr
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS ocpl_graph ocpl_sampling ocpl_tsr
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/planners.cpp
  src/factories.cpp
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
