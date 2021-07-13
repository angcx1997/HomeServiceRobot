#!/bin/sh
# xterm  -e  "source /opt/ros/melodic/setup.bash; roscore" & 
# sleep 5
xterm  -e  "export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_gazebo turtlebot3_myworld.launch" &
sleep 5
xterm  -e  "roslaunch turtlebot3_gazebo turtlebot3_myworld_rviz.launch" &
sleep 5
xterm  -e  "export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" &
sleep 5
xterm -e "roslaunch gmapping gmapping_demo.launch"