#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Setting parameters of pick up and drop off goals
  double dropOffX;
  n.getParam("drop_off_goal_x" , dropOffX);
  double dropOffY;
  n.getParam("drop_off_goal_y" , dropOffY);
  double pickUpX;
  n.getParam("pick_up_goal_x" , pickUpX);
  double pickUpY;
  n.getParam("pick_up_goal_y" , pickUpY);
  
  // Define a position and orientation for the robot to reach
  ROS_INFO("Going to Pick Up Goal");
  goal.target_pose.pose.position.x = pickUpX;
  goal.target_pose.pose.position.y = pickUpY;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, base has reached pick up zone");
  else
    ROS_INFO("The base failed to reach pick up zone");

  sleep(5);
  
    // Define a position and orientation for the robot to reach
  ROS_INFO("Going to Drop Off Goal");
  goal.target_pose.pose.position.x = dropOffX;
  goal.target_pose.pose.position.y = dropOffY;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, has reached drop off zone");
  else
    ROS_INFO("The base failed to reach drop off zone");
  
  sleep(5);
  
  return 0;
}