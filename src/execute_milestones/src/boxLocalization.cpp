#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
//#include <tf2/LinearMath/Quaternion.h>

std::string marker_id3, marker_id4, marker_id5, marker_id6, marker_id7, marker_id = " ";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "box_localization");

  ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("tf", 1000, checkTfTree);
  nh.getParam("/box1/marker_id", marker_id);

  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  // visualization_msgs::Marker marker;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped transformStamped2;
  double x, y, z, q_x, q_y, q_z, q_w, x2, y2, z2 = 0;

  ros::Rate rate(10.0);
  while (nh.ok())
  {

    try
    {
      transformStamped = tfBuffer.lookupTransform("map", "fiducial_4", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try
    {
      transformStamped2 = tfBuffer.lookupTransform("map", "fiducial_5", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = transformStamped.transform.translation.x;
    marker.pose.position.y = transformStamped.transform.translation.y;
    marker.pose.position.z = transformStamped.transform.translation.z;
    marker.pose.orientation.x = transformStamped.transform.rotation.x;
    marker.pose.orientation.y = transformStamped.transform.rotation.y;
    marker.pose.orientation.z = transformStamped.transform.rotation.z;
    marker.pose.orientation.w = transformStamped.transform.rotation.w;
    marker.scale.x = 0.1;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "camera_link";
    marker2.header.stamp = ros::Time();
    marker2.ns = "my_namespace2";
    marker2.id = 0;
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = transformStamped2.transform.translation.x;
    marker2.pose.position.y = transformStamped2.transform.translation.y;
    marker2.pose.position.z = transformStamped2.transform.translation.z;
    marker2.pose.orientation.x = transformStamped2.transform.rotation.x;
    marker2.pose.orientation.y = transformStamped2.transform.rotation.y;
    marker2.pose.orientation.z = transformStamped2.transform.rotation.z;
    marker2.pose.orientation.w = transformStamped2.transform.rotation.w;
    marker2.scale.x = 0.1;
    marker2.scale.y = 0.05;
    marker2.scale.z = 0.05;
    marker2.color.a = 1.0; // Don't forget to set the alpha!
    marker2.color.r = 0.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;

    /*x = transformStamped.transform.translation.x;
    y = transformStamped.transform.translation.y;
    z = transformStamped.transform.translation.z;*/

    vis_pub.publish(marker);
    vis_pub.publish(marker2);
  }
  // ros::spin();
  rate.sleep();
  return 0;
}