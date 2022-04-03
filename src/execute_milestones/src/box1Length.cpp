#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "box1Length");

  ros::NodeHandle nh;

  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
;
  geometry_msgs::TransformStamped transformStamped;

  ros::Rate rate(10.0);
  while (nh.ok()){

    try{
      transformStamped = tfBuffer.lookupTransform("map", "fiducial_4" , ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    } 
    
    visualization_msgs::Marker markerLength;
    markerLength.header.frame_id = "map";
    markerLength.header.stamp = ros::Time();
    markerLength.ns = "box1Length";
    markerLength.id = 0;
    markerLength.type = visualization_msgs::Marker::SPHERE;
    markerLength.action = visualization_msgs::Marker::ADD;
    markerLength.pose.position.x = transformStamped.transform.translation.x;
    markerLength.pose.position.y = transformStamped.transform.translation.y;
    markerLength.pose.position.z = transformStamped.transform.translation.z;
    markerLength.pose.orientation.x = transformStamped.transform.rotation.x;
    markerLength.pose.orientation.y = transformStamped.transform.rotation.y;
    markerLength.pose.orientation.z = transformStamped.transform.rotation.z;
    markerLength.pose.orientation.w = transformStamped.transform.rotation.w;
    markerLength.scale.x = 0.1;
    markerLength.scale.y = 0.1;
    markerLength.scale.z = 0.1;
    markerLength.color.a = 1.0; // Don't forget to set the alpha!
    markerLength.color.r = 1.0;
    markerLength.color.g = 0.0;
    markerLength.color.b = 0.0;


    vis_pub.publish( markerLength );

  }
  //ros::spin();
  rate.sleep();
  return 0;
}