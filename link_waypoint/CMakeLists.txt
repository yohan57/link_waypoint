cmake_minimum_required(VERSION 2.8.3)
project(link_waypoint)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch
)

add_executable(gps_waypoint src/gps_waypoint.cpp)
add_executable(gps_waypoint_mapping src/gps_waypoint_mapping.cpp)
add_executable(calibrate_heading src/calibrate_heading.cpp)

target_link_libraries(gps_waypoint ${catkin_LIBRARIES})
target_link_libraries(gps_waypoint_mapping ${catkin_LIBRARIES})
target_link_libraries(calibrate_heading ${catkin_LIBRARIES})

catkin_metapackage()
