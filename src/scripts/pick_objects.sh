#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/my_robot/worlds/prasunworld.world " &
#xterm  -e  " roslaunch my_robot world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/my_robot/maps/map.yaml " &
#xterm  -e  " roslaunch my_robot amcl.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " & 
sleep 5
xterm  -e  " rosrun pick_objects pick_objects.cpp " 