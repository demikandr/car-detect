#include "point_type.h"
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Utils.h"
#include<iostream>
#include<string>
#include<algorithm>
#include "CylindricProjection.h"
#include "ConfigWrapper.h"
#include <car_detect/TrackedObject.h>
#include <queue>
#include <tf/tfMessage.h>
#include "Clusterizer.h"

// А потом сохранять box-
std::string version="0.004";


int
main (int argc, char** argv)
{

    ros::init (argc, argv, "tracker_fast");
    Clusterizer clusterizer;
  // Initialize ROS
    ROS_DEBUG("Hello Listen DEBUG");
    ROS_INFO("Hello Listen INFO");
    ROS_WARN("Hello Listen WARN");
    ros::NodeHandle nh;
    std::cerr << "version" << version << std::endl;
    // Create a ROS subscriber for the input point cloud

    ros::Subscriber sub = nh.subscribe("input", 1, &Clusterizer::operator(), &clusterizer);
    ros::Subscriber subOdometry = nh.subscribe("tf", 1, &Clusterizer::updateOdometry, &clusterizer);

    // Create a ROS publisher for the output point cloud
    clusterizer.pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1000);
    clusterizer.bboxPub = nh.advertise<car_detect::TrackedObject>("/tracker_fast/object", 1000);
    // Spin
    ros::spin ();
}
