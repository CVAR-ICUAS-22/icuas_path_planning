cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME laserscan_to_map)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_DEPENDENCIES 
  roscpp  
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf
  laser_geometry
  image_transport
  # Eigen3
  # mav_msgs
  # tf2_ros
  # geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PROJECT_DEPENDENCIES}
)

find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
  include/${PROJECT_NAME}
)

set(SOURCE_FILES
  src/laserscan_to_map.cpp
)

add_executable(${PROJECT_NAME}_node src/laserscan_to_map_node.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})