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
#include <deque>
#include <tf/tfMessage.h>
#include <pcl_ros/transforms.h>
#include "Clusterizer.h"
#include <pcl_ros/impl/transforms.hpp>
#include <boost/optional.hpp>
#include <visualization_msgs/Marker.h>

// А потом сохранять box-
std::string version="0.004";


std::deque<Detections> history;

int maxHistorySize = 3;


void addDetectionsToHistory(const Detections& detections) {
    assert(maxHistorySize > 0);
    if (history.size() == maxHistorySize) {
        history.pop_front();
    }
    history.push_back(detections);
}

bool checkDetectionNotZero(const BBox& detection) {
//    if (detection.xMax == 0) {
//        return false;
//    }
//    if (detection.yMax == 0) {
//        return false;
//    }
//    if (detection.zMax == 0) {
//        return false;
//    }
//    if (detection.xMin == 0) {
//        return false;
//    }
//    if (detection.yMin == 0) {
//        return false;
//    }
//    if (detection.zMin == 0) {
//        return false;
//    }
    return true;
}

boost::optional<visualization_msgs::Marker> getVelocity(const std::vector<boost::optional<BBox>>& detections, const int detection_idx) { // ordered from lastes to the most recent
    std::vector<float> x, y, z, idx;
    std::vector<visualization_msgs::Marker> markers;
    for (int i = 0; i < detections.size(); ++i) {
        if (detections[i]) {
            idx.push_back(i);
            assert(checkDetectionNotZero(detections[i].get()));
            x.push_back(detections[i]->xCenter);
            y.push_back(detections[i]->yCenter);
            z.push_back(detections[i]->zCenter);
        }
    }
    if (idx.size() > 1) {
        std::cerr << "At least one velocity thereis\n";
        std::cerr << '\t' << x.back() << ' ' << x.front()
                  << '\t' << y.back() << ' ' << y.front()
                  << '\t' << z.back() << ' ' << z.front() << std::endl;
//        std::cerr << x.size() << std::endl;
//        for (auto detection:detections) {
//            if (detection) {
//                std::cerr << detection->xMin << '\t' << detection->xMax << std::endl;
//            }
//        }
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header.frame_id = "/car_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "velocity";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = detection_idx;
        marker.color.a = 255;
        marker.color.r = 255;

        geometry_msgs::Point p1, p2;
        p1.x = x.back();
        p1.y = y.back();
        p1.z = z.back();
        p2.x = p1.x + (x.back() - x.front()) / (idx.back() - idx.front()) * 10;
        p2.y = p1.y + (y.back() - y.front()) / (idx.back() - idx.front()) * 10;
        p2.z = p1.z + (z.back() - z.front()) / (idx.back() - idx.front()) * 10;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        return marker;
    } else {
        return boost::none;
    }
}

float getVelocityFromMarker(const visualization_msgs::Marker& marker) {
    geometry_msgs::Point p1, p2;
    p1 = marker.points[0];
    p2 = marker.points[1];
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) +
                     (p2.y - p1.y) * (p2.y - p1.y) +
                     (p2.z - p1.z) * (p2.z - p1.z)
    );
}

std::vector<boost::optional<visualization_msgs::Marker>> getDetectionsFromHistory(const float threshold) {
    assert(!history.empty());
    std::vector<boost::optional<visualization_msgs::Marker>> markers;
    int i = 0;
    for (BBox& detection: history.back().detections) {
        std::vector<boost::optional<BBox>> similarDetections;
        for (const Detections oldDetections: history) {
            boost::optional<BBox> closestBBox = oldDetections.findClosestDetectionO(detection, threshold);
            if (closestBBox) {
                assert(checkDetectionNotZero(closestBBox.get()));
            }
            similarDetections.push_back(closestBBox);
        }
        markers.push_back(getVelocity(similarDetections, i));
        if (markers.back()) {
            detection.speed = getVelocityFromMarker(markers.back().get());
            detection.markeredPredok = detection.speed > 2;
            for (const auto& similarDetection: similarDetections) {
                if (similarDetection && similarDetection->markeredPredok) {
                    detection.markeredPredok = true;
                }
            }
        }
        i++;
//        if len(similarDetections )
    }
    return markers;
}


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

    pcl::copyPointCloud(orderedCloud, globalCoordsCloud);
//    pcl_ros::transformPointCloud(orderedCloud, globalCoordsCloud, clusterizer.transform);
    Detections detections(globalCoordsCloud, clusters, clustersNumber);
    addDetectionsToHistory(detections);
    std::vector<boost::optional<visualization_msgs::Marker>> velocities = getDetectionsFromHistory(clusterizer.config.threshold);

    std::vector<bool> mask = clusterizer.filterDetections(detections);

    pcl::PointCloud<pcl::PointXYZRGBA> colored_cloud = clusterizer.colourClusters(globalCoordsCloud, clusters, mask);
    //        std::cerr << cloud.points[15].x <<  '\t' << cloud.points[15].y << '\t' << cloud.points[15].z << '\t' << cloud.points.size() << std::endl;
    //        std::cerr << cloud.points[30015].x <<  '\t' << cloud.points[30015].y << '\t' << cloud.points[30015].z << '\t' << cloud.points.size() << std::endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(colored_cloud, output);
    clusterizer.pub.publish(output);
    car_detect::TrackedObjects trackedObjects = detections.getTrackedObjects(mask);
    clusterizer.bboxPub.publish(trackedObjects);
    std::cerr << "Start publishing velocities\n";
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    clusterizer.velocityPub.publish(marker);
    for (int i = 0; i < velocities.size(); ++i) {
        if (mask[i + 2] and velocities[i] and history.back().detections[i].markeredPredok) {
            std::cerr << i << '\t' << velocities[i]->scale.x
                    << '\t' << velocities[i]->scale.y
                    << '\t' << velocities[i]->scale.z << std::endl;
            clusterizer.velocityPub.publish(velocities[i].get());
        }
    }
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
    clusterizer.velocityPub = nh.advertise<visualization_msgs::Marker>("/tracker_fast/velocity", 1000);
    // Spin
    ros::spin ();
}
