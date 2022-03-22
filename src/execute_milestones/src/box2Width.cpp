#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "box2Width");

  ros::NodeHandle nh;

  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
;
  geometry_msgs::TransformStamped transformStamped;

  ros::Rate rate(10.0);
  while (nh.ok()){

    try{
      transformStamped = tfBuffer.lookupTransform("map", "marker_id8" , ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    } 
    
    visualization_msgs::Marker markerWidth;
    markerWidth.header.frame_id = "map";
    markerWidth.header.stamp = ros::Time();
    markerWidth.ns = "box2Width";
    markerWidth.id = 0;
    markerWidth.type = visualization_msgs::Marker::SPHERE;
    markerWidth.action = visualization_msgs::Marker::ADD;
    markerWidth.pose.position.x = transformStamped.transform.translation.x;
    markerWidth.pose.position.y = transformStamped.transform.translation.y;
    markerWidth.pose.position.z = transformStamped.transform.translation.z;
    markerWidth.pose.orientation.x = transformStamped.transform.rotation.x;
    markerWidth.pose.orientation.y = transformStamped.transform.rotation.y;
    markerWidth.pose.orientation.z = transformStamped.transform.rotation.z;
    markerWidth.pose.orientation.w = transformStamped.transform.rotation.w;
    markerWidth.scale.x = 0.1;
    markerWidth.scale.y = 0.1;
    markerWidth.scale.z = 0.1;
    markerWidth.color.a = 1.0; // Don't forget to set the alpha!
    markerWidth.color.r = 0.0;
    markerWidth.color.g = 1.0;
    markerWidth.color.b = 1.0;


    vis_pub.publish( markerWidth );

  }
  //ros::spin();
  rate.sleep();
  return 0;
}