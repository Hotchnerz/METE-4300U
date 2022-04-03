#include <stdio.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "restrictArea");
    ros::start();

    // get node handle
    ros::NodeHandle n;
    ros::Rate loopRate(1.0);
    std::string topicName = "boxScan";

    ros::Publisher demoPublisher = n.advertise<pcl::PointCloud<pcl::PointXYZ> >(topicName.c_str(),10);

    ROS_INFO("Publishing point cloud on topic \"%s\" once every second.", topicName.c_str());


    while (ros::ok())
    {
        // create point cloud object
        pcl::PointCloud<pcl::PointXYZ> myCloud;

        myCloud.header.frame_id = "/map";


        // fill cloud with random points
        /*for (int v=0; v<1000; ++v)
        {
            pcl::PointXYZ newPoint;
            newPoint.x = (rand() * 100.0) / RAND_MAX;
            newPoint.y = (rand() * 100.0) / RAND_MAX;
            newPoint.z = (rand() * 100.0) / RAND_MAX;
            myCloud.points.push_back(newPoint);
        }*/

            pcl::PointXYZ newPoint;
            newPoint.x = 0.5;
            newPoint.y = 0.5;
            newPoint.z = 0.5;
            myCloud.points.push_back(newPoint);

        // publish point cloud
        demoPublisher.publish(myCloud.makeShared());

        // pause for loop delay
        loopRate.sleep();
    }

    return 1;
}