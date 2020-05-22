# link_waypoint
waypoint for LINK inspired by waypoint_nav
https://github.com/nickcharron/waypoint_nav

## Control_base
Since we're using arduino board as a motor control board, arduino control code is added (subscribes /cmd_vel).

## Local Planner
changed base_local_planner to teb_local_planner (for car like robot).

## OSM
Added osm-cartography pkg to visualize map

## Issues
Still needs some tuning with parameters.
