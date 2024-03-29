cmake_minimum_required(VERSION 3.0.2)
project(ocpl_planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()



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
  src/planner_base.cpp
  src/acro_planner_samplers.cpp
  src/acro_planner.cpp
  src/sample_space.cpp
  src/factories.cpp
  src/io.cpp
  src/oriolo.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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

  # catkin_add_gtest(${PROJECT_NAME}_test_oriolo test/test_oriolo.cpp)

  # target_link_libraries(${PROJECT_NAME}_test_oriolo
  #   ${PROJECT_NAME}
  #   ${catkin_LIBRARIES}
  # )

  catkin_add_gtest(${PROJECT_NAME}_test_planner_base test/test_planner_base.cpp)

  target_link_libraries(${PROJECT_NAME}_test_planner_base
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

endif()
