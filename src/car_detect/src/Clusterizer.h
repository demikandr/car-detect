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

using namespace NCylindricProjection;
class Clusterizer {
public:
    ros::Publisher pub, bboxPub, velocityPub;
    ConfigWrapper config;
    std::queue<car_detect::TrackedObjects> history;
    tf::Transform transform;


    Clusterizer() {
    }

    bool isOutlier(const NCylindricProjection::CylindricProjection& cloud, int x, int y) const {
        return config.plant_threshold < Utils::variance(cloud.cylindricProjection[(x + N_COLUMNS - 1) % N_COLUMNS][y],
                                                        cloud.cylindricProjection[x][y],
        cloud.cylindricProjection[(x + 1) % N_COLUMNS][y]);
    }
    bool isPlant(const NCylindricProjection::CylindricProjection& cloud, int x, int y) const {
        std::vector<std::pair<int, int>> candidates;
        candidates.push_back(std::make_pair((x + 1) % N_COLUMNS, y));
        candidates.push_back(std::make_pair((x + N_COLUMNS - 1) % N_COLUMNS, y));
        candidates.push_back(std::make_pair((x + 2) % N_COLUMNS, y));
        candidates.push_back(std::make_pair((x + N_COLUMNS - 2) % N_COLUMNS, y));
        candidates.push_back(std::make_pair(x, y));
        if (y > 0) {
            candidates.push_back(std::make_pair(x, y-1));
        }
        if (y + 1 < N_RINGS) {
            candidates.push_back(std::make_pair(x, y + 1));
        }
        int outliersInNeighbourhoodNumber = 0;
        for (auto candidate: candidates) {
            outliersInNeighbourhoodNumber += isOutlier(cloud, candidate.first, candidate.second);
        }
        float outliersInNeighbourhoodPercent = (float) outliersInNeighbourhoodNumber / (float) candidates.size();
        return config.oinp < outliersInNeighbourhoodPercent;
    }

    std::vector<std::vector<int>> markPlants(const NCylindricProjection::CylindricProjection& cloud) const {
        std::vector<std::vector<int>> clusters(NCylindricProjection::N_COLUMNS, std::vector<int>(NCylindricProjection::N_RINGS));
        for (int i = 0; i < NCylindricProjection::N_COLUMNS; ++i) {
            for (int j = 0; j < NCylindricProjection::N_RINGS; ++j) {
                clusters[i][j] = isPlant(cloud, i, j);
            }
        }
        return clusters;
    }

    bool isGroundEdge(const int& x, const int& y) const {
        assert(false);
    }

    bool isGroundEdge(const velodyne_pointcloud::PointOffsetIRL& x, const velodyne_pointcloud::PointOffsetIRL& y,
                      const velodyne_pointcloud::PointOffsetIRL& x1, const velodyne_pointcloud::PointOffsetIRL& y1) const {

        return (std::abs(Utils::oxyAngle(x, x1) - Utils::oxyAngle(y, y1)) < config.alpha_ground); // &&
//                ((Utils::distance(x, x1) + Utils::distance(y, y1)) / std::min(Utils::distance(x, x1), Utils::distance(y, y1)) < 4);
    }


    void markGroundDfs(const NCylindricProjection::CylindricProjection& cloud,
                  std::vector<std::vector<int>>* groundMaskPtr, int i, int j) const {
        assert(cloud.mask[i][j]);
        assert(j + 1< NCylindricProjection::N_RINGS);
        std::vector<std::vector<int>>& groundMask = *groundMaskPtr;
        groundMask[i][j] = 1;
        auto tryStep = [&](int x, int y) {
            assert(y + 1 < NCylindricProjection::N_RINGS);
            if ((cloud.mask[x][y]) && (groundMask[x][y] == 0) && (cloud.cylindricProjection[x][y].z < -1) &&
                    isGroundEdge(cloud.cylindricProjection[i][j], cloud.cylindricProjection[x][y],
                                                                                cloud.cylindricProjection[i][j+1], cloud.cylindricProjection[x][y+1])) {// &&
//                    ((std::abs(y - j) > 0) || (Utils::distance(cloud.cylindricProjection[i][j],  cloud.cylindricProjection[x][y]) < 0.5))) {
                markGroundDfs(cloud, groundMaskPtr, x, y);
            }
        };
        if (j > 0) {
            tryStep(i, j-1);
        }
        if (j < NCylindricProjection::N_RINGS - 2) {
            tryStep(i, j + 1);
        }
        tryStep((i + 1) % NCylindricProjection::N_COLUMNS, j);
        tryStep((i + NCylindricProjection::N_COLUMNS - 1) % NCylindricProjection::N_COLUMNS, j);
    }

    std::vector<std::vector<int>> markGround(const NCylindricProjection::CylindricProjection& cloud) const {
        ROS_DEBUG("start markGround");
//        std::vector<std::vector<int>> groundMask(NCylindricProjection::N_COLUMNS, std::vector<int>(NCylindricProjection::N_RINGS, 0));
        std::vector<std::vector<int>> groundMask = markPlants(cloud);
        const std::vector<std::vector<int>>& mask = cloud.mask;
        int counter = 0;
        for (size_t j = 0; j < mask[0].size(); ++j) {
            for (size_t i = 0; i < mask.size(); ++i) {

                if (mask[i][j] and (j + 1 < NCylindricProjection::N_RINGS) and (cloud.cylindricProjection[i][j].z < -1) and
                        ((j == 0) or (mask[i][j-1] == 0)) and
                        (Utils::oxyAngleCos(velodyne_pointcloud::PointOffsetIRL(), cloud.cylindricProjection[i][j]) < 0.5))
                { //}} && !groundMask[Utils::safe_idx(i-1)]) {
                    assert(j + 1 < NCylindricProjection::N_RINGS);
                    markGroundDfs(cloud, &groundMask, i, j);

                }
            }
        }
//        std::cerr << "NGroundPoints\t" << counter << std::endl;
        ROS_DEBUG("end markGround");
        return groundMask;
    }


    bool isEdge(const velodyne_pointcloud::PointOffsetIRL& x, const velodyne_pointcloud::PointOffsetIRL& y) const {
        return ((abs(Utils::oxyAngleCos(x, y)) < config.alpha))
               and (Utils::distance(x, y) < config.min_distance);
//               and (Utils::distance(x, y) / std::min(Utils::l2(x), Utils::l2(y)) < config.min_relative_distance);
    }

    int dfs(const int pointIdx,  std::vector<std::vector<int>>* clustersPtr, const std::vector<std::vector<int>>& edges,
            const NCylindricProjection::CylindricProjection& cylindricProjection, const int clusterIdx) const {
        std::vector<std::vector<int>>& clusters = *clustersPtr;

//        std::cerr << "Point Idx? " << pointIdx << std::endl;
        clusters[pointIdx / NCylindricProjection::N_RINGS][pointIdx % NCylindricProjection::N_RINGS] = clusterIdx;
        const velodyne_pointcloud::PointOffsetIRL  point = cylindricProjection.getPointByIdx(pointIdx);
        int clusterSize = 1;
//        std::cerr << "probably not " << pointIdx << std::endl;
        for (int i = 0; i < edges[pointIdx].size(); ++i) {
            int nextPointIdx = edges[pointIdx][i];
            const velodyne_pointcloud::PointOffsetIRL& nextPoint = cylindricProjection.getPointByIdx(nextPointIdx);
            if ((nextPointIdx >= 0) && (clusters[nextPointIdx / NCylindricProjection::N_RINGS][nextPointIdx % NCylindricProjection::N_RINGS] == 0) && isEdge(point, nextPoint)) {
                clusterSize += dfs(nextPointIdx, clustersPtr, edges, cylindricProjection, clusterIdx);
            }
        }
        return clusterSize;
    }

    std::pair<std::vector<std::vector<int>>, int> clusterize(const NCylindricProjection::CylindricProjection& cylindricProjection) const {
        ROS_DEBUG("start clusterize");
        std::vector<std::vector<int>> edges = cylindricProjection.getEdges();

        std::vector<std::vector<int>> clusters = markGround(cylindricProjection); // 0 is not a ground, 1 is a ground

        std::vector<std::vector<int>> visited(cylindricProjection.getScansNumber(), std::vector<int>(cylindricProjection.getScanLength(), 0));
        int nextClusterIdx = 2;

        std::cerr << "start dfs loop" << std::endl;
        std::vector<int> clusterSizes(2, -1);
        for (int i = 0; i < NCylindricProjection::N_COLUMNS; ++i) {
            for (int j = 0; j < NCylindricProjection::N_RINGS; ++j) {
                if (cylindricProjection.mask[i][j]) {
                    int idx = NCylindricProjection::toIdx(i,j);
                    if (clusters[i][j] == 0) {
                        clusterSizes.push_back(dfs(idx, &clusters, edges, cylindricProjection, nextClusterIdx));
                        nextClusterIdx++;
                    }
                }
            }
        }
        // fulter clusters
        std::cerr << "end dfs loop" << std::endl;
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
        for (int i = 0; i < NCylindricProjection::N_COLUMNS; ++i) {
            for (int j = 0; j < NCylindricProjection::N_RINGS; ++j) {
                assert((cylindricProjection.mask[i][j] == 0) or (clusters[i][j] > 0));
                clusters[i][j] = filteredClusterIndices[clusters[i][j]];
            }
        }

        ROS_DEBUG("end clusterize");
        return std::make_pair(clusters, clustersNumber); //clustersNumber);
    }

    void colourPoint(pcl::PointXYZRGBA& point, const int cluster, const std::vector<bool>& mask) const {
        if (cluster == 0) { // nothing is found

            point.r = 255;
            if (config.show_no_detects) {
                point.a = 255;
            } else {
                point.a = 1;
            }
        } else if (mask[cluster] == false) {
            point.r = 255;
            point.g = 255;
            if (config.show_false_detects) {
                point.a = 255;
            } else {
                point.a = 1;
            }
        } else if (cluster == 1) {
            point.g = 255;
            if (config.show_ground) {
                point.a = 255;
            } else {
                point.a = 1;
            }
        } else {
            point.r = cluster * 53 % 256;
            point.g = cluster * 29 % 256;
            point.b = cluster * 71 % 256;
            if (config.show_true_detects) {
                point.a = 255;
            } else {
                point.a = 1;
            }
//            point.b = 255;
        }
    }


    pcl::PointCloud<pcl::PointXYZRGBA> colourClusters(const NCylindricProjection::CylindricProjection& cylindricProjection,
                                                     const std::vector<std::vector<int>>& clusters,
                                                     const std::vector<bool>& mask) const {
        ROS_DEBUG("start colourClusters");

        pcl::PointCloud<pcl::PointXYZRGBA> coloured_cloud;


        coloured_cloud.header.frame_id = "velo_link";
        for (int i = 0; i < NCylindricProjection::N_COLUMNS; ++i) {
            for (int j = 0; j < NCylindricProjection::N_RINGS; ++j) {
                if (cylindricProjection.mask[i][j]) {
                    pcl::PointXYZRGBA point;
                    point.x = cylindricProjection.cylindricProjection[i][j].x;
                    point.y = cylindricProjection.cylindricProjection[i][j].y;
                    point.z = cylindricProjection.cylindricProjection[i][j].z;
                    colourPoint(point, clusters[i][j], mask);
                    coloured_cloud.push_back(point);
                }
            }
        }
//        std::cerr << "Before initialization" << coloured_cloud.height << '\t' << coloured_cloud.width << '\t' << coloured_cloud.size() << std::endl;
//        coloured_cloud.height = 1;
//        coloured_cloud.width = coloured_cloud.size();

        ROS_DEBUG("end colourClusters");
        return coloured_cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBA> colourClusters(const NCylindricProjection::CylindricProjection& cloud,
                                                     const std::vector<std::vector<int>>& clusters) const {
        const std::vector<bool> mask(cloud.size(), true);
        pcl::PointCloud<pcl::PointXYZRGBA> result = colourClusters(cloud, clusters, mask);
        ROS_DEBUG("WAT");
        return result;
    }

    pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> restoreOrder(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud) const {
        return cloud;
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
            std::vector<float> dimensions{
                                          detection.yMax - detection.yMin, detection.xMax - detection.xMin};
            std::sort(dimensions.begin(), dimensions.end());
            dimensions.push_back(detection.zMax - detection.zMin);
            mask.push_back(
//                    (dimensions[0] < config.detections0_filter_upper_bound) and
//                    (dimensions[0] > config.detections0_filter_lower_bound) and
//                    (dimensions[1] < config.detections1_filter_upper_bound) and
//                    (dimensions[1] > config.detections1_filter_lower_bound) and
//                    (dimensions[2] < config.detections_z_filter_upper_bound) and
//                    (dimensions[2] > config.detections_z_filter_lower_bound)
                true
            );
        }
//        std::vector<bool> mask(detections.detections.size() + 2, true);
        return mask;
    }
};