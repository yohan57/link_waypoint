# link_waypoint
waypoint for LINK inspired by husky navigation
http://wiki.ros.org/husky_navigation

## Control_base
Since we're using arduino board as a motor control board, arduino control code is added (subscribes /cmd_vel).

## Local Planner
changed base_local_planner to teb_local_planner (for car like robot).

## Issues
Still needs some tuning with parameters.
