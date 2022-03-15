#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
//#include <tf2/LinearMath/Quaternion.h>

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

  tf2_ros::Buffer tfBuffer2;
  tf2_ros::TransformListener tfListener2(tfBuffer2);
  //geometry_msgs::PoseStamped poseStamped;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped transformStamped2;
  double x, y, z, q_x, q_y, q_z, q_w, x2, y2, z2;


  ros::Rate rate(10.0);
  while (node.ok()){

    try{
      transformStamped = tfBuffer.lookupTransform("map", "marker_id3", ros::Time(0)); 
    x = transformStamped.transform.translation.x;
    y = transformStamped.transform.translation.y;
    z = transformStamped.transform.translation.z;

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    } 

    try{
    transformStamped2 = tfBuffer2.lookupTransform("map", "marker_id4", ros::Time(0)); 

    x2 = transformStamped2.transform.translation.x;
    y2 = transformStamped2.transform.translation.y;
    z2 = transformStamped2.transform.translation.z;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    } 
    

    //std::cout << transformStamped << std::endl;



    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;

    std::cout << "-------------------------------" << std::endl;

    std::cout << "x: " << x2 << std::endl;
    std::cout << "y: " << y2 << std::endl;
    std::cout << "z: " << z2 << std::endl;
    


    transformStamped.transform.rotation.x = q_x;
    transformStamped.transform.rotation.y = q_y;
    transformStamped.transform.rotation.z = q_z;
    transformStamped.transform.rotation.w = q_w;

  
    /*int time = 3;
    //std::cout << "Box 1 WRT to Map Origin:" << std::endl;
    ROS_INFO_THROTTLE(time, "Box 1 WRT to Map Origin:");
    ROS_INFO_THROTTLE(time, "-----------------------------------");
    ROS_INFO_THROTTLE(time, "XYZ Coordinates:");
    ROS_INFO_STREAM_THROTTLE(time, "x: " << x);
    ROS_INFO_STREAM_THROTTLE(time, "y: " << y);
    ROS_INFO_STREAM_THROTTLE(time, "z: " << z);
    ROS_INFO_THROTTLE(time, " ");
    ROS_INFO_THROTTLE(time, "Quaternion Coordinates:");
    ROS_INFO_STREAM_THROTTLE(time, "x: " << q_x);
    ROS_INFO_STREAM_THROTTLE(time, "y: " << q_y);
    ROS_INFO_STREAM_THROTTLE(time, "z: " << q_z);
    ROS_INFO_STREAM_THROTTLE(time, "z: " << q_w);*/


    /*geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);*/
  }
  
  rate.sleep();
  return 0;
}