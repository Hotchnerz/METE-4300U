#include <stdio.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "restrictArea");
    ros::start();

    // get node handle
    ros::NodeHandle n;
    ros::Rate loopRate(1.0);
    std::string topicName = "boxScan";

    ros::Publisher pointCloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >(topicName.c_str(), 10);

    ROS_INFO("Publishing point cloud on topic \"%s\" once every second.", topicName.c_str());

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    while (ros::ok())
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

        // create point cloud object
        pcl::PointCloud<pcl::PointXYZ> arucoCloud;

        arucoCloud.header.frame_id = "/map";

        // fill cloud with random points
        /*for (int v=0; v<1000; ++v)
        {
            pcl::PointXYZ newPoint;
            newPoint.x = (rand() * 100.0) / RAND_MAX;
            newPoint.y = (rand() * 100.0) / RAND_MAX;
            newPoint.z = (rand() * 100.0) / RAND_MAX;
            myCloud.points.push_back(newPoint);
        }*/

        pcl::PointXYZ arucoPoint;
        arucoPoint.x = transformStamped.transform.translation.x;
        arucoPoint.y = transformStamped.transform.translation.y;
        arucoPoint.z = transformStamped.transform.translation.z;
        arucoCloud.points.push_back(arucoPoint);

        // publish point cloud
        pointCloud_pub.publish(arucoCloud.makeShared());

        // pause for loop delay
        loopRate.sleep();
    }

    return 1;
}