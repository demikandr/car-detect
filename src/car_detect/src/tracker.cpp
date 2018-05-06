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
#include "ConfigWrapper.h"
#include "Detections.h"
#include <ros/console.h>
#include <car_detect/TrackedObject.h>
#include <queue>
#include <tf/tfMessage.h>

// А потом сохранять box-
std::string version="0.004";
class Clusterizer {
public:
    ros::Publisher pub, bboxPub;
    ConfigWrapper config;
    std::queue<car_detect::TrackedObject> history;
    
    void operator() (const sensor_msgs::PointCloud2ConstPtr& input)
    {

        // Create a container for the data.

        pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> cloud;
        pcl::fromROSMsg(*input, cloud);
        //    pcl::
        const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> orderedCloud  = restoreOrder(cloud);
        auto clusterization = clusterize(orderedCloud);
        const std::vector<int> clusters = clusterization.first;
        const int clustersNumber = clusterization.second;
        Detections detections(orderedCloud, clusters, clustersNumber);
        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud = colourClusters(orderedCloud, clusters);
        //        std::cerr << cloud.points[15].x <<  '\t' << cloud.points[15].y << '\t' << cloud.points[15].z << '\t' << cloud.points.size() << std::endl;
        //        std::cerr << cloud.points[30015].x <<  '\t' << cloud.points[30015].y << '\t' << cloud.points[30015].z << '\t' << cloud.points.size() << std::endl;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(colored_cloud, output);
        pub.publish(output);
        car_detect::TrackedObject trackedObject = detections.detections[0].getTrackedObject();
        bboxPub.publish(trackedObject);
    }
    Clusterizer() {
    }
    std::vector<int> markGround(const Utils& utils, const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) {
        ROS_DEBUG("start markGround");
        std::vector<std::vector<int>> edges(cloud.size());
        std::vector<int> groundMask(cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i) {
            if (i % 10000 == 0) {
                std::cerr << i << std::endl;
            }
    //        std::cerr << i << '\t' << cloud[i].ring << std::endl;
            if ((utils.isLowestLevel(i) or ((i > 0) && (groundMask[i-1] == 1)))
                 and !utils.isHighestLevel(i) and (utils.oxyAngleCos(i, i+1) > 0.8) ) { //}} && !groundMask[utils.safe_idx(i-1)]) {
                groundMask[i] = 1;
            }
        }
        ROS_DEBUG("end markGround");
        return groundMask;
    }

    bool isEdge(const Utils& utils, const int pointIdx1, const int pointIdx2) {
        return ((abs(utils.oxyAngleCos(pointIdx1, pointIdx2)) < config.alpha) or (utils.distance(pointIdx1, pointIdx2) < 0.1))
                and (utils.distance(pointIdx1, pointIdx2) / std::min(utils.l2(pointIdx1), utils.l2(pointIdx2)) < 0.5 / 2.);
    }

    int dfs(const int pointIdx,  std::vector<int>* clustersPtr, const std::vector<std::vector<int>>& edges, const int clusterIdx, const Utils& utils) {
        std::vector<int>& clusters = *clustersPtr;

        clusters[pointIdx] = clusterIdx;
        int clusterSize = 1;
        for (int i = 0; i < edges[pointIdx].size(); ++i) {
            int nextPointIdx = edges[pointIdx][i];
            if ((nextPointIdx >= 0) && (clusters[nextPointIdx] == 0) && isEdge(utils, pointIdx, nextPointIdx)) {
                clusterSize += dfs(nextPointIdx, clustersPtr, edges, clusterIdx, utils);
            }
        }
        return clusterSize;
    }

    std::pair<std::vector<int>, int> clusterize(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) {
        ROS_DEBUG("start clusterize");
        const Utils utils(cloud);
        ROS_DEBUG("start cylindricProjectiionCreation");
        const CylindricProjection& cylindricProjection(cloud);
        std::vector<std::vector<int>> edges = cylindricProjection.getEdges();

        std::vector<int> clusters = markGround(utils, cloud); // 0 is not a ground, 1 is a ground

        std::vector<std::vector<int>> visited(cylindricProjection.getScansNumber(), std::vector<int>(cylindricProjection.getScanLength(), 0));
        int nextClusterIdx = 2;

        ROS_DEBUG("start dfs loop");
        std::vector<int> clusterSizes(2, -1);
        for (int i = 0; i < cloud.size(); ++i) {
            if (clusters[i] == 0) {
                clusterSizes.push_back(dfs(i, &clusters, edges, nextClusterIdx, utils));
                nextClusterIdx++;
            }
        }
        // fulter clusters
        ROS_DEBUG("end dfs loop");
        std::vector<int> filteredClusterIndices{0,1};
        int clustersNumber = 2;
        for (int i = 2; i < clusterSizes.size(); ++i) {
            if (clusterSizes[i] < config.min_cluster_size) {
                filteredClusterIndices.push_back(0);
            } else {
                filteredClusterIndices.push_back(clustersNumber);
                clustersNumber++;
            }
        }
        ROS_INFO("Clusters number %d", clustersNumber);
        for (int i = 0; i < cloud.size(); ++i) {
                clusters[i] = filteredClusterIndices[clusters[i]];
        }

        ROS_DEBUG("end clusterize");
        return std::make_pair(clusters, clustersNumber);
    }

    void colourPoint(pcl::PointXYZRGB& point, const int cluster) {
        if (cluster == 0) {
            point.r = 255;
        } else {
            point.r = cluster * 53 % 256;
            point.g = 100 + cluster * 29 % 256;
            point.b = 200 + cluster * 71 % 256;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB> colourClusters(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud,
            const std::vector<int>& clusters) {
        ROS_DEBUG("start colourClusters");

        pcl::PointCloud<pcl::PointXYZRGB> coloured_cloud;
        pcl::copyPointCloud(cloud, coloured_cloud);
        for (int i = 0; i < coloured_cloud.size(); ++i) {
            colourPoint(coloured_cloud[i], clusters[i]);
        }

        ROS_DEBUG("end colourClusters");
        return coloured_cloud;
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
    void updateOdometry(const tf::tfMessageConstPtr& messagePtr) {

    }
};

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
