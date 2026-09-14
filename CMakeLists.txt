cmake_minimum_required(VERSION 2.8.3)
project(far_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosgraph_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  topic_tools
  visibility_graph_msg
  pcl_ros
  pluginlib
  rviz
)

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()
add_definitions(-DQT_NO_KEYWORDS)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES goalpoint_rviz_plugin
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  rosgraph_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  topic_tools
  pcl_ros
  pluginlib
  rviz
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
  ${QT_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
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

add_executable(semantic_voxel_to_far_terrain
  src/semantic_voxel_to_far_terrain.cpp)
add_dependencies(semantic_voxel_to_far_terrain
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(livox_custom_decoder src/livox_custom_decoder.cpp)
add_dependencies(livox_custom_decoder
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(far_scan_input_adapter src/scan_input_adapter.cpp)
add_dependencies(far_scan_input_adapter
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Keep the upstream RViz class name (rviz/GoalPointTool) so the existing
# default.rviz configuration loads the tool without any manual GUI setup.
add_library(goalpoint_rviz_plugin SHARED
  src/goalpoint_tool.cpp
  include/far_planner/goalpoint_tool.h)
add_dependencies(goalpoint_rviz_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(semantic_voxel_to_far_terrain ${catkin_LIBRARIES})
target_link_libraries(livox_custom_decoder ${catkin_LIBRARIES})
target_link_libraries(far_scan_input_adapter
  livox_custom_decoder ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(goalpoint_rviz_plugin
  ${catkin_LIBRARIES} ${QT_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_livox_custom_decoder
    test/test_livox_custom_decoder.cpp)
  if(TARGET test_livox_custom_decoder)
    target_link_libraries(test_livox_custom_decoder
      livox_custom_decoder ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} semantic_voxel_to_far_terrain
                livox_custom_decoder far_scan_input_adapter
                goalpoint_rviz_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(FILES FAR_Planner_Local3D语义Voxel接口改造方案.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
)

install(FILES far_planner_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
