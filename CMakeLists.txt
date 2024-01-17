cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME icuas_path_planner)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_DEPENDENCIES
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  mav_msgs
  icuas_msgs
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${PROJECT_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

set(SOURCE_FILES
  src/icuas_path_planner.cpp
)

catkin_package(
  INCLUDE_DIRS include include/${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_node src/icuas_path_planner_node.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
