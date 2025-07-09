cmake_minimum_required(VERSION 3.0.2)
project(eco_track)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(PkgConfig) 
find_package(OpenCV REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)
include_directories(${GST_INCLUDE_DIRS})
find_package(Threads REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES eco_track
#  CATKIN_DEPENDS roscpp geometry_msgs std_msgs sensor_msgs cv_bridge image_transport
#  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}/src/EcoTracker-master
)


file(GLOB ECO_SRC_FILES#找对应路径下的可执行文件，放入${ECO_SRC_FILES}
  src/EcoTracker-master/*.cc
)
list(REMOVE_ITEM ECO_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/EcoTracker-master/eco_unittest.cc")

add_executable(eco_tracker_node 
scripts/eco_tracker_node.cpp
${ECO_SRC_FILES}
)

target_link_libraries(eco_tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
   ${GST_LIBRARIES}
)

