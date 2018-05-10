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
#include <car_detect/TrackedObjects.h>
#include <queue>
#include <tf/tfMessage.h>
#include <pcl_ros/transforms.h>
#include "Clusterizer.h"
#include <pcl_ros/impl/transforms.hpp>

// А потом сохранять box-
std::string version="0.004";

void callback(const Clusterizer& clusterizer, const sensor_msgs::PointCloud2ConstPtr& input)
{

    // Create a container for the data.

    pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> cloud;
    pcl::fromROSMsg(*input, cloud);

    //    pcl::
    const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> orderedCloud  = clusterizer.restoreOrder(cloud);
    auto clusterization = clusterizer.clusterize(orderedCloud);
    const std::vector<int> clusters = clusterization.first;
    const int clustersNumber = clusterization.second;
    // ATTENTION! CHANGE COORDINATE SYSTEM
    pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> globalCoordsCloud;
    pcl_ros::transformPointCloud(orderedCloud, globalCoordsCloud, clusterizer.transform);
    Detections detections(globalCoordsCloud, clusters, clustersNumber);
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud = clusterizer.colourClusters(globalCoordsCloud, clusters);
    //        std::cerr << cloud.points[15].x <<  '\t' << cloud.points[15].y << '\t' << cloud.points[15].z << '\t' << cloud.points.size() << std::endl;
    //        std::cerr << cloud.points[30015].x <<  '\t' << cloud.points[30015].y << '\t' << cloud.points[30015].z << '\t' << cloud.points.size() << std::endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(colored_cloud, output);
    clusterizer.pub.publish(output);
    car_detect::TrackedObjects trackedObjects = detections.getTrackedObjects();
    clusterizer.bboxPub.publish(trackedObjects);
}

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

    std::function<void(const sensor_msgs::PointCloud2ConstPtr&)> callback_ = [&](const sensor_msgs::PointCloud2ConstPtr& input){callback(clusterizer, input);};
    ros::Subscriber sub = nh.subscribe("input", 1, &std::function<void(const sensor_msgs::PointCloud2ConstPtr&)>::operator(), &callback_);
    ros::Subscriber subOdometry = nh.subscribe("tf", 1, &Clusterizer::updateOdometry, &clusterizer);

    // Create a ROS publisher for the output point cloud
    clusterizer.pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1000);
    clusterizer.bboxPub = nh.advertise<car_detect::TrackedObjects>("/tracker_fast/object", 1000);
    // Spin
    ros::spin ();
}
