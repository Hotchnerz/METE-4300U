#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskActionResult.h>
#include <frontier_exploration/ExploreTaskAction.h>

void exploreStatusCallback(const frontier_exploration::ExploreTaskActionResult::ConstPtr& msg)
{
  int status = msg->status.status;

  if (status == 3) {
  ROS_INFO("Exploration Complete. Returning to start position!");

  } else {
      ROS_INFO("Navigation Aborted. Error Occured");
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nav_monitor");
  ros::NodeHandle n;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> unboundEx("explore_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  unboundEx.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  frontier_exploration::ExploreTaskGoal goal1;
  goal1.explore_boundary.header.frame_id = "map";
  goal1.explore_boundary.polygon.points;

  goal1.explore_center.header.frame_id = "map";
  goal1.explore_center.point.x = 0;
  goal1.explore_center.point.y = 0;
  goal1.explore_center.point.z = 0;

  unboundEx.sendGoal(goal1);


  //Subscriber Part of the Node
  ros::Subscriber sub = n.subscribe("explore_server/result", 10, exploreStatusCallback);

  

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