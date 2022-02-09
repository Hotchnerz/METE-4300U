#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PolygonStamped.h>
#include <frontier_exploration/ExploreTaskGoal.h>

int main(int argc, char** argv)
{
 ros::init(argc, argv, "send_exploration_goal");

 actionlib::SimpleActionClient<frontier_exploration::ExploreTaskGoal> client("explore_server", true);
 //Client client("explore_server", true);
 client.waitForServer();
 frontier_exploration::ExploreTaskGoal goal;

 geometry_msgs::PolygonStamped boundary;
 geometry_msgs::Point32 p1, p2, p3, p4;

 p1.x = 2.0; p1.y = 2.0; p1.z = 0.0;
 p2.x = 2.0; p2.y = -2.0; p2.z = 0.0;
 p3.x = -2.0; p3.y = -2.0; p3.z = 0.0;
 p4.x = -2.0; p4.y = 2.0; p4.z = 0.0;

 // defining the boundary polygon here
 boundary.header.seq = 1;
 boundary.header.stamp = ros::Time::now();
 boundary.header.frame_id = "map";
 boundary.polygon.points.reserve(4);
boundary.polygon.points.push_back(p1);
boundary.polygon.points.push_back(p2);
boundary.polygon.points.push_back(p3);
boundary.polygon.points.push_back(p4);

 // defining the explore center here
 geometry_msgs::PointStamped start_point;
 start_point.point.x = 0.0; start_point.point.y = 0.0; start_point.point.z = 0.0;

 goal.explore_center = start_point;
 goal.explore_boundary = boundary;
 client.sendGoal(goal);
}