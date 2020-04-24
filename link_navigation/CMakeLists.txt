cmake_minimum_required(VERSION 2.8.3)
project(link_navigation)

# Find catkin
find_package(catkin REQUIRED)

catkin_package()

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install move_base_config files
<<<<<<< HEAD
install(DIRECTORY params
=======
install(DIRECTORY move_base_config
>>>>>>> 5ee7db8591f21ac05426d8e3daabca1147f24895
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install the rviz files
install(FILES multi_robot.rviz single_robot.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
<<<<<<< HEAD

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_navigation
#  CATKIN_DEPENDS hector_slam husky_customization move_base outdoor_waypoint_nav robot_localization
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
=======
>>>>>>> 5ee7db8591f21ac05426d8e3daabca1147f24895
