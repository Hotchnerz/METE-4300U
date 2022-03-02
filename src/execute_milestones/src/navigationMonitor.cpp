#include <ros/ros.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskActionResult.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>

//Initialize global variables
int nav_status = 0;

//Subscriber callback for frontier_exploration result
void exploreStatusCallback(const frontier_exploration::ExploreTaskActionResult::ConstPtr& msg)
{

  //Create the action client to send move_base goals
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> returnHome("move_base", true);
  
  //Wait for move_base to start
  returnHome.waitForServer();

  //Create a goal message (homeGoal) and set the following parameters for the message type.
  move_base_msgs::MoveBaseGoal homeGoal;

  //Store pointer data of explore_server result integer into nav_status
  nav_status = msg->status.status;

  /* For Reference on status meanings
  actionlib_msgs/GoalStatus status
  uint8 PENDING=0
  uint8 ACTIVE=1
  uint8 PREEMPTED=2
  uint8 SUCCEEDED=3
  uint8 ABORTED=4
  uint8 REJECTED=5
  uint8 PREEMPTING=6
  uint8 RECALLING=7
  uint8 RECALLED=8
  uint8 LOST=9
  */

  //Check if nav_status is correct
  if (nav_status == 3) {

  //Update user on milestone status
  ROS_INFO("Exploration Complete. Returning to start position!");

  //State point of reference for data to contained in this message and state pose information to be sent over goal
  homeGoal.target_pose.header.frame_id = "map";
  homeGoal.target_pose.pose.position.x = 0;
  homeGoal.target_pose.pose.position.y = 0;
  homeGoal.target_pose.pose.orientation.w = 1;

  //Send goal to move_base
  returnHome.sendGoal(homeGoal);

  } else {
      //If navigation did not complete successfully, update user on failed milestone
      ROS_INFO("Navigation Aborted. Error Occurred");
  }

}

//Subscriber callback for move_base result
void homeStatusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& homemsg)
{
  //Check if home_status is the correct status, SUCCEED
  int home_status = homemsg->status.status;

//If frontier exploration completed successfully, then update user on milestone status related to returning to inital pose depending on the status flag
if (nav_status == 3){
  if (home_status == 3) {
  ROS_INFO("Milestone Complete. Home Position Reached.");

  } else if (home_status == 2){
      ROS_INFO("Home Navigation In Progress. Observe the terminal of slam_launch for navigation updates...");
  } else {
      ROS_INFO("Navigation Aborted. Error Occurred");
  }
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nav_monitor");
  ros::NodeHandle n;
  ros::NodeHandle n2;

  //Create the action client for unbounded exploration
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> unboundEx("explore_server", true);

  //Wait for explore_server to start
  ROS_INFO("Waiting for explore_server to start.");

  //will wait for infinite time
  unboundEx.waitForServer(); 

  ROS_INFO("explore_server started, sending goal.");

  //Create a goal message (frontierGoal) and set the following parameters for the message type.
  frontier_exploration::ExploreTaskGoal frontierGoal;

  //State point of reference for data to contained in this message
  frontierGoal.explore_boundary.header.frame_id = "map";

  //Send an empty boundary polygon for unbounded exploration
  frontierGoal.explore_boundary.polygon.points;

  //State point of reference for data to contained in this message
  frontierGoal.explore_center.header.frame_id = "map";


  //State the initial exploration point in the bounded polygon
  //This is 0,0,0 as unbounded exploration is performed.
  frontierGoal.explore_center.point.x = 0;
  frontierGoal.explore_center.point.y = 0;
  frontierGoal.explore_center.point.z = 0;

  // send this goal to explore_server
  unboundEx.sendGoal(frontierGoal);


  //Subscriber Part of the Node. nav_sub listens to the result topic of explore_server with a message queue size of 10 and calls the explore status callback
  //home_sub listens to result topic of move_base with the same queue size as nav_sub with a similar callback function
  ros::Subscriber nav_sub = n.subscribe("explore_server/result", 10, exploreStatusCallback);
  ros::Subscriber home_sub = n2.subscribe("move_base/result", 10, homeStatusCallback);

  ros::spin();

  return 0;
}