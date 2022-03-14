#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "box_localization");

  ros::NodeHandle node;

  /*ros::service::waitForService("spawn");
  ros::ServiceClient spawner =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);*/

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map", "marker_id3",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    double x, y, z, q_x, q_y, q_z, q_w = 0;

    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    transformStamped.transform.rotation.x = q_x;
    transformStamped.transform.rotation.y = q_y;
    transformStamped.transform.rotation.z = q_z;
    transformStamped.transform.rotation.w = q_w;

    ROS_INFO("Box 1 WRT to Map Origin:");
    ROS_INFO("-----------------------------------");
    ROS_INFO("XYZ Coordinates:");
    ROS_INFO("x: ", x);
    ROS_INFO("y: ", y);
    ROS_INFO("z: ", z);
    ROS_INFO(" ");
    ROS_INFO("Quaternion Coordinates:");
    ROS_INFO("x: ", q_x);
    ROS_INFO("y: ", q_y);
    ROS_INFO("z: ", q_z);
    ROS_INFO("z: ", q_w);



    /*geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep(); */

    

  }
  return 0;
};