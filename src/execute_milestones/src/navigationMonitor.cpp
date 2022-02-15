#include <ros/ros.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskActionResult.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>

//int status = 0;
int debug = 0;

void exploreStatusCallback(const frontier_exploration::ExploreTaskActionResult::ConstPtr& msg)
{

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> returnHome("move_base", true);
  returnHome.waitForServer();
  move_base_msgs::MoveBaseGoal homeGoal;

  int status = msg->status.status;

  if (status == 3) {
  ROS_INFO("Exploration Complete. Returning to start position!");
  homeGoal.target_pose.header.frame_id = "map";

  homeGoal.target_pose.pose.position.x = 0;
  homeGoal.target_pose.pose.position.y = 0;

  homeGoal.target_pose.pose.orientation.w = 1;

  returnHome.sendGoal(homeGoal);

  } else {
      ROS_INFO("Navigation Aborted. Error Occured");
  }

  /*if (status == 3) {
  ROS_INFO("Exploration Complete. Returning to start position!");

  } else {
      ROS_INFO("Navigation Aborted. Error Occured");
  }*/
}

void homeStatusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& homemsg)
{

  move_base_msgs::MoveBaseGoal homeGoal;

  int home_status = homemsg->status.status;

  if (home_status == 3) {
  ROS_INFO("Milestone Complete. Home Postition Reached");


  } else {
      ROS_INFO("Navigation Mission Aborted. Error Occured While Returning to Home.");
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nav_monitor");
  ros::NodeHandle n;
  ros::NodeHandle n2;


  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> unboundEx("explore_server", true);
  //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> returnHome("move_base", true);

  ROS_INFO("Waiting for explore_server to start.");
  // wait for the action server to start
  unboundEx.waitForServer(); //will wait for infinite time
  //returnHome.waitForServer();

  ROS_INFO("explore_server started, sending goal.");
  // send a goal to the action
  frontier_exploration::ExploreTaskGoal frontierGoal;
  frontierGoal.explore_boundary.header.frame_id = "map";
  frontierGoal.explore_boundary.polygon.points;

  frontierGoal.explore_center.header.frame_id = "map";
  frontierGoal.explore_center.point.x = 0;
  frontierGoal.explore_center.point.y = 0;
  frontierGoal.explore_center.point.z = 0;

  unboundEx.sendGoal(frontierGoal);

  //move_base_msgs::MoveBaseGoal homeGoal;

  //Subscriber Part of the Node
  ros::Subscriber nav_sub = n.subscribe("explore_server/result", 10, exploreStatusCallback);
  ros::Subscriber home_sub = n2.subscribe("move_base/result", 10, homeStatusCallback);

  /*while (debug==0){
    std::cout << "Status: " << status << std::endl;

  }*/

  /*while (status == 0) {
  if (status == 3) {
  ROS_INFO("Exploration Complete. Returning to start position!");
  homeGoal.target_pose.header.frame_id = "map";

  homeGoal.target_pose.pose.position.x = 0;
  homeGoal.target_pose.pose.position.y = 0;

  homeGoal.target_pose.pose.orientation.w = 1;

  returnHome.sendGoal(homeGoal);

  } else {
      ROS_INFO("Navigation Aborted. Error Occured");
  }
  }*/




  

  //actionlib_tutorials::FibonacciGoal goal;
  //goal.order = 20;
  //ac.sendGoal(goal);

  //wait for the action to return
  /*bool finished_before_timeout = unboundEx.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = unboundEx.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else {
    ROS_INFO("Action did not finish before the time out.");
  } */

  ros::spin();

  //exit
  return 0;
}