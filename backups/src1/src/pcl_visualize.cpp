#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "utility.h"

class cloudHandler: public ParamServer
{
public:
    cloudHandler()
    : viewer("Cloud Viewer")
    {
        // pcl_sub = nh.subscribe("cloud_deskewed", 10, &cloudHandler::cloudCB, this);
        pcl_sub = nh.subscribe(pointCloudTopic, 10, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(input, cloud);

        viewer.showCloud(cloud.makeShared());
    }

    void timerCB(const ros::TimerEvent&)
    {
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize");

    cloudHandler handler;

    ros::spin();

    return 0;
}
