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
//#include <pcl/point_cloud.h>
//#include <pcl/impl/point_types.hpp>
#include "CylindricProjection.h"

ros::Publisher pub;
std::string version="0.004";

std::vector<int> markGround(const Utils& utils, const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) {
    std::vector<std::vector<int>> edges(cloud.size());
    std::vector<int> groundMask(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (i % 10000 == 0) {
            std::cerr << i << std::endl;
        }
//        std::cerr << i << '\t' << cloud[i].ring << std::endl;
        if ((utils.isLowestLevel(i) or ((i > 0) && (groundMask[i-1] == true)))
             and !utils.isHighestLevel(i) and (utils.oxyAngleCos(i, i+1) > 0.8) ) { //}} && !groundMask[utils.safe_idx(i-1)]) {
            groundMask[i] = true;
        }
    }
    return groundMask;
}

bool isEdge(const Utils& utils, const int pointIdx1, const int pointIdx2) {
    return abs(utils.oxyAngleCos(pointIdx1, pointIdx2)) < 0.9;
}
void dfs(const int pointIdx,  std::vector<int>* clustersPtr, const std::vector<std::vector<int>> edges, const int clusterIdx, const Utils& utils) {
    std::vector<int> clusters = *clustersPtr;
    clusters[pointIdx] = clusterIdx;
    for (int i = 0; i < edges[pointIdx].size(); ++i) {
        int nextPointIdx = edges[pointIdx][i];
        if ((nextPointIdx >= 0) && (clusters[nextPointIdx] == 0) && isEdge(utils, pointIdx, nextPointIdx)) {
            dfs(edges[pointIdx][i], clustersPtr, edges, clusterIdx, utils);
        }
    }
}

std::vector<int> clusterize(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) {
    const Utils utils(cloud);
    const CylindricProjection& cylindricProjection(cloud);
    std::vector<std::vector<int>> edges(cylindricProjection.getScanLength(), std::vector<int>(4, -1));

    std::vector<int> clusters = markGround(utils, cloud); // 0 is not a ground, 1 is a ground

    std::vector<std::vector<int>> visited(cylindricProjection.getScansNumber(), std::vector(cylindricProjection.getScanLength(), 0));
    int nextClusterIdx = 0;
    for (int i = 0; i < cloud.size(); ++i) {
        if (clusters[i] == 0) {
            dfs(i, &clusters, edges, nextClusterIdx, utils);
            nextClusterIdx++;
        }
    }
    return clusters;
}

void colourPoint(pcl::PointXYZRGB& point, const int cluster) {
    point.r = cluster * 53 % 256;
    point.g =100 +  cluster * 29 % 256;
    point.b = 200 + cluster * 71 % 256;
}
pcl::PointCloud<pcl::PointXYZRGB> colourClusters(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud,
        const std::vector<int>& clusters) {
    pcl::PointCloud<pcl::PointXYZRGB> coloured_cloud;
    pcl::copyPointCloud(cloud, coloured_cloud);
    for (int i = 0; i < coloured_cloud.size(); ++i) {
        colourPoint(coloured_cloud[i], clusters[i]);
    }
}

pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> restoreOrder(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) {
    std::vector<velodyne_pointcloud::PointOffsetIRL> buffer{cloud[0]};
    pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> orderedCloud(cloud);
    orderedCloud.clear();
    for (int i = 1; i < cloud.size(); ++i) {
//        std::cerr << i << '\t' << cloud[i].ring << std::endl;
        if (cloud[i].ring == 1 +  buffer.back().ring) {
            buffer.push_back(cloud[i]);
        } else if ((cloud[i].ring > 1 + buffer.back().ring) and (cloud[i].ring > orderedCloud.back().ring)) {
//            std::cerr << "O\t" << cloud[i].ring << std::endl;
            orderedCloud.push_back(cloud[i]);
        } else {
//            std::cerr << "clear buffer" << std::endl;
            for (velodyne_pointcloud::PointOffsetIRL& point: buffer) {
//                std::cerr <<  "B\t" << point.ring << std::endl;
                orderedCloud.push_back(point);
            }
            buffer.clear();
            buffer.push_back(cloud[i]);
        }
    }
    for (velodyne_pointcloud::PointOffsetIRL& point: buffer) {
        orderedCloud.push_back(point);
    }
    std::cerr << "Cloud is ordered" << std::endl;
    return orderedCloud;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  // Create a container for the data.

    pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> cloud;
    pcl::fromROSMsg(*input, cloud);
    //    pcl::
    std::vector<int> clusters = clusterize(restoreOrder(cloud));
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud = colourClusters(cloud, clusters);
//        std::cerr << cloud.points[15].x <<  '\t' << cloud.points[15].y << '\t' << cloud.points[15].z << '\t' << cloud.points.size() << std::endl;
//        std::cerr << cloud.points[30015].x <<  '\t' << cloud.points[30015].y << '\t' << cloud.points[30015].z << '\t' << cloud.points.size() << std::endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(colored_cloud, output);
    pub.publish(output);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "tracker_fast");
  ros::NodeHandle nh;
    std::cerr << "version" << version << std::endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1000);

  // Spin
  ros::spin ();
}
