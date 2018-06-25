//
// Created by demikandr on 6/23/18.
//

#include "MainLoop.h"
#include "CloudMask"
#include "PlantsMask"
#include "GroundMask"
#include "mask/CommonMask.h"

#include <ros/ros.h>

void MainLoop::callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    MyPointCloud cloud(input);
    PlantsMask plantsMask = CommonMask::getEmptyMasking(cloud);
    GroundMask groundMask = CommonMask::getEmptyMasking(cloud);
    CloudMask plantsAndGroundMask = plantsMask.getUnionWith(groundMask);
    Segmentation clusterization = getSegmentation(cloud, plantsAndGroundMask);
    cloudPub.publish(cloud.toPclMsg());
    plantsMaskPub.publish(cloud.toPclMsg(plantsMask));
    groundMaskPub.publish(cloud.toPclMsg(groundMask));
    clustarizationPub.publish(cloud.toPclMsg(Clusterization));
}

MainLoop::MainLoop():
        nh(),
        config(),
        pub(CNodeName + "/output", 1),
        bboxPub(CNodeName + "/object", 1),
        velocityPub(CNodeName + "/velocity", 1) {
    std::function<void(const sensor_msgs::PointCloud2ConstPtr&)> callback_ = [&](const sensor_msgs::PointCloud2ConstPtr& input){callback(clusterizer, input);};
    ros::Subscriber sub = nh.subscribe("input", 1, &std::function<void(const sensor_msgs::PointCloud2ConstPtr&)>::operator(), &callback_);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    bboxPub = nh.advertise<car_detect::TrackedObjects>("/tracker_fast/object", 1);
    velocityPub = nh.advertise<visualization_msgs::Marker>("/tracker_fast/velocity", 1);
}