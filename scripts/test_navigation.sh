#!/bin/sh
xterm  -e  "export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_gazebo turtlebot3_myworld.launch" &
sleep 5
xterm  -e  "roslaunch turtlebot3_gazebo turtlebot3_myworld_rviz.launch" &
sleep 5
xterm -e "roslaunch turtlebot3_navigation amcl_demo.launch"