# HomeServiceRobot

The goal of this project is to design a environment for the mobile-base robot (in our application, turtlebot3 is used) and program it to performance localization and navigate itself to a pick-up point to pick-up virtual object projected in rviz then drop-it back to preset location.
1. Initially show the marker at the pickup zone.
2. Hide the marker once your robot reach the pickup zone.
3. Wait 5 seconds to simulate a pickup.
4. Show the marker at the drop off zone once your robot reaches it.


### Package Used
- `slam_gmapping` provides laser based Grid-based FastSLAM algorithms. It helps to build 2D occupancy grid map of the environment by feeding in laser scan measurement and odometry value. The map will be updated as the robot moves and throught the sensory information collected.
- `ROS_Navigation/amcl` is probabilistic localization system to move a robot in 2D. It used adaptive Monte Carlo localization approach to track the pose of robot against a known map. Tuning parameter is required and in our application, due to limited sensory noise, the parameter is tuned to be relatively low. Initial localization and accurate help to speed up the convegence process.
- `ROS_Navigation/move_base` helps interacting with `navigation stack` and output appropriate velocity to navigate the robot to desired goal pose. Trajectory rollout is help to perform local paht planning while Dijkstra's algorithms is used for global path planning.
- `turtlebot3`, `turtlebot3_msgs` and `turtlebot3_simulation` contain the turtlebot model and gazebo simuation. All 3 packages needs to be install for the package to run smoothly in simulation. For this project,turtlebot3 burger is used but different model used would not affect the performance. 
- `pick_up`  node sends the pick up and drop off point to `turtlebot3`. The point is sent to `move_base` server and then perform subsequent path planning.
- `add_marker` node helps to visualize the pick-up and drop-off point by allocating a shape onto `rviz` generated maps. It subscribe to `pick_up` for action planning.

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

### Rviz Interface
![](https://github.com/angcx1997/HomeServiceRobot/blob/main/img/Screenshot%20from%202021-07-16%2014-53-02.png)

### To run in a new environment
You can perform the node using you own environment,
1. Import your world into `maps`
2. Perform a gmapping on your environment by running script `test_slam.sh`
3. After finish mapping out the environment, save the map by running `rosrun map_server map_saver -f myMap` in terminal
4. Put the `.pgm` and `.yaml` file into `maps`
5. The initial pose in `amcl_demo.launch` need to be the same as `turtlebot3_myworld.launch`, so that robot is able to localize itself at the right place when the program run.
6. To find out the pose that you wish to go, you could use `2D nav goal` in rviz, and read the pose by `rostopic echo /amcl_pose`
