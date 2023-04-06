cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME path_planner)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_DEPENDENCIES 
  roscpp  
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  cv_bridge
  tf
  laser_geometry
  image_transport
  message_generation
  message_runtime
  grid_map_core
  grid_map_cv
)

find_package(catkin REQUIRED COMPONENTS
  ${PROJECT_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(SOURCE_FILES
  src/path_planner.cpp
)

add_service_files(DIRECTORY srv
  FILES
  setGoalPoint.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_node src/path_planner_node.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h")

# install(DIRECTORY migration_rules
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# add_custom_target(${PROJECT_NAME}
#             SOURCES ${DRONEMSGS_HEADER_FILES}
#             )
