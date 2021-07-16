# HomeServiceRobot

The goal of this project is to design a environment for the mobile-base robot (in our application, turtlebot3 is used) and program it to performance localization and navigate itself to a pick-up point to pick-up virtual object projected in rviz then drop-it back to preset location.
- slam_gmapping is used in mapping environment
- ROS navigation stack is used in performing motion-planning and AMCL as local path planner.
- pick_object node publish the desired location to move_base
- add_markers node publish the virtual object into rviz environment

### Installation
Execute the following commands in the terminal to set up workspace and clone the this repo
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ sudo apt-get update
$ cd ~/catkin_ws/src
$ git clone https://github.com/angcx1997/HomeServiceRobot.git
```
Then build it
```
$ cd ~/catkin_ws/
$ catkin_make
```

### Launching
`xterm` is used to keep the convenience of running a single command to launch all nodes.
To install `xterm`
```
$ sudo apt-get install xterm
```

To run the program
```
cd ~/catkin_ws/src/scripts
./home_service.sh
```

`turtlebot3_myworld.launch`: launch turtlebot3 inside gazebo world

`turtlebot3_myworld_rviz.launch`: launch rviz to visualize the topics

`amcl_demo.launch`: launch ros navigation stack



