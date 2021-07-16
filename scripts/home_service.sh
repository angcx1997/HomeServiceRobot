#!/bin/sh
xterm -e "export TURTLEBOT3_MODEL=burger; 
roslaunch turtlebot3_gazebo turtlebot3_myworld.launch" &
sleep 10
xterm  -e  "roslaunch turtlebot3_gazebo turtlebot3_myworld_rviz.launch" &
sleep 5
xterm -e "roslaunch turtlebot3_navigation amcl_demo.launch" &
sleep 5
xterm -e "rosparam load $(pwd)/../pick_objects/config/params.yaml; 
rosrun add_markers add_markers" &
sleep 5
xterm -e "rosparam load $(pwd)/../pick_objects/config/params.yaml;
rosrun pick_objects pick_objects"