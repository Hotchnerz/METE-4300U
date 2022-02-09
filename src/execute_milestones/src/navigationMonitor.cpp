#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskActionResult.h>
#include <frontier_exploration/ExploreTaskAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nav_monitor");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> unboundEx("explore_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  unboundEx.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  frontier_exploration::ExploreTaskGoal goal;
  goal.explore_boundary.header.frame_id = "map";
  goal.explore_boundary.polygon.points;

  goal.explore_center.header.frame_id = "map";
  goal.explore_center.point.x = 0;
  goal.explore_center.point.y = 0;
  goal.explore_center.point.z = 0;

  unboundEx.sendGoal(goal);

  

  //actionlib_tutorials::FibonacciGoal goal;
  //goal.order = 20;
  //ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = unboundEx.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = unboundEx.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}