// PCL specific includes
#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Utils.h"
#include<iostream>
#include<string>
#include<algorithm>
#include "CylindricProjection.h"
#include "ConfigWrapper.h"
#include "Detections.h"
#include <ros/console.h>
#include <car_detect/TrackedObjects.h>
#include <queue>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

class Clusterizer {
public:
    ros::Publisher pub, bboxPub;
    ConfigWrapper config;
    std::queue<car_detect::TrackedObjects> history;
    tf::Transform transform;


    Clusterizer() {
    }

    bool isGroundEdge(const Utils& utils, int i, int j) const {
        return utils.oxyAngleCos(i, j) > config.alpha_ground;
    }
    std::vector<int> markGround(const Utils& utils, const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) const {
        ROS_DEBUG("start markGround");
        std::vector<int> groundMask(cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i) {
            if ((utils.isLowestLevel(i) or ((i > 0) && (groundMask[i-1] == 1)))
                and !utils.isHighestLevel(i) and isGroundEdge(utils, i, i+1) ) { //}} && !groundMask[utils.safe_idx(i-1)]) {
                groundMask[i] = 1;
            }
        }
        ROS_DEBUG("end markGround");
        return groundMask;
    }
    std::vector<int> markGround(const Utils& utils, const CylindricProjection& cloud) const {
        ROS_DEBUG("start markGround");
        std::vector<int> groundMask(cloud.nPoints);
        const std::vector<std::vector<int>>& mask = cloud.mask;
        const std::vector<std::vector<int>>& indices = cloud.indices;
        for (size_t i = 0; i < mask.size(); ++i) {
            for (size_t j = 0; j < mask[i].size(); ++j) {
//                std::cerr << i << '\t' << j << '\t' << indices[i][j] <<  std::endl;
                if (mask[i][j] and
                        ((j == 0) or (mask[i][j-1] == 0) or (groundMask[indices[i][j - 1]] == 1) or
                        ((i > 0) and groundMask[indices[i-1][j]]) or // TODO(demikandr) case of i == 0
                        ((i + 1 < mask.size()) and groundMask[indices[i+1][j]])) and // TODO(demikandr) case of i + 1 == mask.size()
                            ((j < mask[i].size()) and mask[i][j + 1] == 1 and isGroundEdge(utils, indices[i][j], indices[i][j+1])))
                     { //}} && !groundMask[utils.safe_idx(i-1)]) {
                    groundMask[indices[i][j]] = 1;
                }
            }
        }
        ROS_DEBUG("end markGround");
        return groundMask;
    }

    bool isEdge(const Utils& utils, const int pointIdx1, const int pointIdx2) const {
        return ((abs(utils.oxyAngleCos(pointIdx1, pointIdx2)) < config.alpha) or (utils.distance(pointIdx1, pointIdx2) < 0.1))
               and (utils.distance(pointIdx1, pointIdx2) / std::min(utils.l2(pointIdx1), utils.l2(pointIdx2)) < 0.5 / 2.);
    }

    int dfs(const int pointIdx,  std::vector<int>* clustersPtr, const std::vector<std::vector<int>>& edges, const int clusterIdx, const Utils& utils) const {
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

    std::pair<std::vector<int>, int> clusterize(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) const {
        ROS_DEBUG("start clusterize");
        const Utils utils(cloud);
        ROS_DEBUG("start cylindricProjectiionCreation");
        const CylindricProjection& cylindricProjection(cloud);
        std::vector<std::vector<int>> edges = cylindricProjection.getEdges();

        std::vector<int> clusters = markGround(utils, cylindricProjection); // 0 is not a ground, 1 is a ground

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

    void colourPoint(pcl::PointXYZRGB& point, const int cluster) const {
        if (cluster == 0) {
            point.r = 255;
        } else {
            point.r = cluster * 53 % 256;
            point.g = 100 + cluster * 29 % 256;
            point.b = 200 + cluster * 71 % 256;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB> colourClusters(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud,
                                                     const std::vector<int>& clusters,
                                                     const std::vector<bool>& mask) const {
        ROS_DEBUG("start colourClusters");

        pcl::PointCloud<pcl::PointXYZRGB> coloured_cloud;
        pcl::copyPointCloud(cloud, coloured_cloud);
        for (int i = 0; i < coloured_cloud.size(); ++i) {
            colourPoint(coloured_cloud[i], clusters[i]);
        }

        ROS_DEBUG("end colourClusters");
        return coloured_cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB> colourClusters(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud,
                                                     const std::vector<int>& clusters) const {
        const std::vector<bool> mask(cloud.size(), true);
        return colourClusters(cloud, clusters, mask);
    }

    pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> restoreOrder(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) const {
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
        return orderedCloud;
    }
    void updateOdometry(const tf::tfMessageConstPtr& messagePtr) {
        assert(messagePtr->transforms.size() == 1);
        if (messagePtr->transforms[0].header.frame_id == "odometric_world") {
            tf::transformMsgToTF(messagePtr->transforms[0].transform, transform);
        }
    }

    std::vector<bool> filterDetections(const Detections& detections) const {
        std::vector<bool> mask(2, true);
        for (const BBox& detection: detections.detections) {
            assert(detection.zMax > detection.zMin);
            mask.push_back(std::max(detection.zMax - detection.zMax,
                                    std::max(detection.yMax - detection.yMin, detection.xMax - detection.xMin)) < config.detections_filter_upper_bound);
        }
        std::vector<bool> mask(detections.detections.size() + 2, true);
        return mask;
    }
};