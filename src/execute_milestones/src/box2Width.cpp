#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "box2Width");
  ros::start();

  ros::NodeHandle nh;

  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  ros::Publisher pointCloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("boxScan", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;

  ros::Rate rate(10.0);
  while (nh.ok())
  {

    try
    {
      transformStamped = tfBuffer.lookupTransform("map", "fiducial_8", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // create point cloud object
    pcl::PointCloud<pcl::PointXYZ> arucoCloud;

    arucoCloud.header.frame_id = "/map";

    pcl::PointXYZ arucoPoint;
    arucoPoint.x = transformStamped.transform.translation.x;
    arucoPoint.y = transformStamped.transform.translation.y;
    arucoPoint.z = transformStamped.transform.translation.z;
    arucoCloud.points.push_back(arucoPoint);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "box2Width";
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
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    // pcl_conversions::toPCL(ros::Time(), arucoCloud.header.stamp);

    pointCloud_pub.publish(arucoCloud.makeShared());
    vis_pub.publish(marker);
  }
  rate.sleep();
  // ros::spin();

  return 0;
}