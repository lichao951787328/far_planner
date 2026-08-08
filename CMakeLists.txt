cmake_minimum_required(VERSION 2.8.3)
project(far_planner)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  visibility_graph_msg
  pcl_ros
  octomap_msgs
  semantic_octomap
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  pcl_ros
  octomap_msgs
  semantic_octomap
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}/SSMI/SSMI-Mapping/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Depedent cpp files
set(SOURCES ${SOURCES}
  src/dynamic_graph.cpp
  src/planner_visualizer.cpp
  src/utility.cpp
  src/graph_planner.cpp
  src/contour_detector.cpp
  src/contour_graph.cpp
  src/map_handler.cpp
  src/scan_handler.cpp
  src/graph_msger.cpp
  src/terrain_planner.cpp
)

## Declare executables
add_executable(${PROJECT_NAME} src/far_planner.cpp ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} semantic_octomap_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(pose_stamped_to_odometry src/pose_stamped_to_odometry.cpp)
add_dependencies(pose_stamped_to_odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_stamped_to_odometry ${catkin_LIBRARIES})

add_executable(pose_stamped_to_point_stamped
  src/pose_stamped_to_point_stamped.cpp)
add_dependencies(pose_stamped_to_point_stamped ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_stamped_to_point_stamped ${catkin_LIBRARIES})

add_executable(navigation_demo_monitor src/navigation_demo_monitor.cpp)
add_dependencies(navigation_demo_monitor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  semantic_octomap_generate_messages_cpp)
target_link_libraries(navigation_demo_monitor ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # Interactive integration test: run the node and then publish /octomap_full.
  add_executable(map_handler_test
    src/map_handler_test.cpp
    src/map_handler.cpp
    src/utility.cpp
    src/contour_detector.cpp
    src/contour_graph.cpp
  )
  add_dependencies(map_handler_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(map_handler_test semantic_octomap_generate_messages_cpp)
  target_link_libraries(map_handler_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

  add_executable(far_planner_bag_test src/far_planner_bag_test.cpp)
  add_dependencies(far_planner_bag_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(far_planner_bag_test semantic_octomap_generate_messages_cpp)
  target_link_libraries(far_planner_bag_test ${catkin_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} pose_stamped_to_odometry
  pose_stamped_to_point_stamped navigation_demo_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
