#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Matrix3x3.h>
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

  //ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map", "marker_id5",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    } 
    
    double x, y, z, q_x, q_y, q_z, q_w, r, p, y = 0;  

    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    transformStamped.transform.rotation.x = q_x;
    transformStamped.transform.rotation.y = q_y;
    transformStamped.transform.rotation.z = q_z;
    transformStamped.transform.rotation.w = q_w;

    tf2::Quaternion q(q_x, q_y, q_z, q_w);
    tf2::Matrix3x3 euler(q);

    euler.getRPY(r, p, y);

  
    int time = 3;
    //std::cout << "Box 1 WRT to Map Origin:" << std::endl;
    ROS_INFO_THROTTLE(time, "Box 1 WRT to Map Origin:");
    ROS_INFO_THROTTLE(time, "-----------------------------------");
    ROS_INFO_THROTTLE(time, "XYZ Coordinates:");
    ROS_INFO_STREAM_THROTTLE(time, "x: " << x);
    ROS_INFO_STREAM_THROTTLE(time, "y: " << y);
    ROS_INFO_STREAM_THROTTLE(time, "z: " << z);
    ROS_INFO_THROTTLE(time, " ");
    ROS_INFO_THROTTLE(time, "Orientation in Fixed Euler Angles:");
    ROS_INFO_STREAM_THROTTLE(time, "Roll: " << r);
    ROS_INFO_STREAM_THROTTLE(time, "Pitch: " << p);
    ROS_INFO_STREAM_THROTTLE(time, "Yaw: " << y);
    //ROS_INFO_STREAM_THROTTLE(time, "z: " << q_w);


    /*geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);*/
  }
  return 0;
}
