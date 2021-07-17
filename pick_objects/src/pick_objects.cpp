#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_object");

  //Declare to retrieved parameters
  ros::NodeHandle nh;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //Publisher to tell whether the robot has reach desired goal
  ros::Publisher marker_pub = nh.advertise<std_msgs::UInt8>("/reach_state", 10);
  std_msgs::UInt8 state;

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  nh.getParam("/pick_up/tx", goal.target_pose.pose.position.x);
  nh.getParam("/pick_up/ty", goal.target_pose.pose.position.y);
  nh.getParam("/pick_up/tz", goal.target_pose.pose.position.z);
  nh.getParam("/pick_up/qx", goal.target_pose.pose.orientation.x);
  nh.getParam("/pick_up/qy", goal.target_pose.pose.orientation.y);
  nh.getParam("/pick_up/qz", goal.target_pose.pose.orientation.z);
  nh.getParam("/pick_up/qw", goal.target_pose.pose.orientation.w);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick-up location");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal{
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base moved towards pick-up location");
    state.data = 2;
    marker_pub.publish(state);
  }
  else{
    ROS_INFO("The base failed to move to pick-up location for some reason");
    // return 0;
  }
  

  // Sleep for 5 s
  ros::Duration(5.0).sleep();


  // Define a position and orientation for the robot to reach
  nh.getParam("/drop_off/tx", goal.target_pose.pose.position.x);
  nh.getParam("/drop_off/ty", goal.target_pose.pose.position.y);
  nh.getParam("/drop_off/tz", goal.target_pose.pose.position.z);
  nh.getParam("/drop_off/qx", goal.target_pose.pose.orientation.x);
  nh.getParam("/drop_off/qy", goal.target_pose.pose.orientation.y);
  nh.getParam("/drop_off/qz", goal.target_pose.pose.orientation.z);
  nh.getParam("/drop_off/qw", goal.target_pose.pose.orientation.w);
  
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop-off location");

  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base moved towards drop-off location");
      state.data = 3;
    marker_pub.publish(state);
    }
  else{
    ROS_INFO("The base failed to move to drop-off location for some reason");
    // return 0;
  }

  // Sleep for 5s
  ros::Duration(5.0).sleep();

  return 0;
}