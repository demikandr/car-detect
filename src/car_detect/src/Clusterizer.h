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
    ros::Publisher pub, bboxPub, velocityPub;
    ConfigWrapper config;
    std::queue<car_detect::TrackedObjects> history;
    tf::Transform transform;


    Clusterizer() {
    }
    bool isGroundEdge(const int& x, const int& y) const {
        assert(false);
    }

    bool isGroundEdge(const velodyne_pointcloud::PointOffsetIRL& x, const velodyne_pointcloud::PointOffsetIRL& y,
                      const velodyne_pointcloud::PointOffsetIRL& x1, const velodyne_pointcloud::PointOffsetIRL& y1) const {
//        if (Utils::oxyAngleCos(x, y) < 0) {
//            std::cerr << "If you see many such messages - you are fucked" <<Utils::oxyAngleCos(x, y) <<  std::endl;
//            std::cerr << x.x << '\t' << x.y << '\t' << x.z << std::endl;
//            std::cerr << y.x << '\t' << y.y << '\t' << y.z << std::endl;
//        }
//        return Utils::oxyAngleCos(x, y) > config.alpha_ground;
        return std::abs(Utils::oxyAngle(x, x1) - Utils::oxyAngle(y, y1)) < NCylindricProjection::PI / 180 * 5;
    }


    void markGroundDfs(const NCylindricProjection::CylindricProjection& cloud,
                  std::vector<std::vector<int>>* groundMaskPtr, int i, int j) const {
        assert(cloud.mask[i][j]);
        assert(j + 1< NCylindricProjection::N_RINGS);
        std::vector<std::vector<int>>& groundMask = *groundMaskPtr;
        groundMask[i][j] = 1;
        auto tryStep = [&](int x, int y) {
            assert(y + 1 < NCylindricProjection::N_RINGS);
            if ((cloud.mask[x][y]) && (groundMask[x][y] == 0) && isGroundEdge(cloud.cylindricProjection[i][j], cloud.cylindricProjection[x][y],
                                                                                cloud.cylindricProjection[i][j+1], cloud.cylindricProjection[x][y+1])) {
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
        std::vector<std::vector<int>> groundMask(NCylindricProjection::N_COLUMNS, std::vector<int>(NCylindricProjection::N_RINGS, 0));
        const std::vector<std::vector<int>>& mask = cloud.mask;
        int counter = 0;
        for (size_t j = 0; j < mask[0].size(); ++j) {
            for (size_t i = 0; i < mask.size(); ++i) {

                if (mask[i][j] and (j + 1 < NCylindricProjection::N_RINGS) and
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

//    std::vector<int> markGround(const NCylindricProjection::CylindricProjection& cloud) const {
//        ROS_DEBUG("start markGround");
//        std::vector<int> groundMask(NCylindricProjection::N_COLUMNS * NCylindricProjection::N_RINGS);
//        const std::vector<std::vector<int>>& mask = cloud.mask;
//        int counter = 0;
//        for (size_t j = 0; j < mask[0].size(); ++j) {
//            for (size_t i = 0; i < mask.size(); ++i) {
//                if (mask[i][j] and
//                        ((j == 0) or (mask[i][j-1] == 0) or (groundMask[NCylindricProjection::toIdx(i,j - 1)] == 1) or
//                        ((i > 0) and groundMask[NCylindricProjection::toIdx(i - 1,j)]) or // TODO(demikandr) case of i == 0
//                        ((i + 1 < mask.size()) and groundMask[NCylindricProjection::toIdx(i + 1,j)])) and // TODO(demikandr) case of i + 1 == mask.size()
////                            (j < mask[i].size()) and
////                                ((j + 2 >= mask[0].size()) || (mask[i][j + 2] == 0) || isGroundEdge(cloud.cylindricProjection[i][j],
//                                                                                                    cloud.cylindricProjection[i][j + 2])))
//                     { //}} && !groundMask[Utils::safe_idx(i-1)]) {
//                    groundMask[NCylindricProjection::toIdx(i,j)] = 1;
//                     counter++;
//
//                }
//            }
//        }
//        std::cerr << "NGroundPoints\t" << counter << std::endl;
//        ROS_DEBUG("end markGround");
//        return groundMask;
//    }

    bool isEdge(const velodyne_pointcloud::PointOffsetIRL& x, const velodyne_pointcloud::PointOffsetIRL& y) const {
        return ((abs(Utils::oxyAngleCos(x, y)) < config.alpha) or (Utils::distance(x, y) < 0.1))
               and (Utils::distance(x, y) / std::min(Utils::l2(x), Utils::l2(y)) < 0.5 / 2.);
    }

    int dfs(const int pointIdx,  std::vector<int>* clustersPtr, const std::vector<std::vector<int>>& edges,
            const NCylindricProjection::CylindricProjection& cylindricProjection, const int clusterIdx) const {
        std::vector<int>& clusters = *clustersPtr;

//        std::cerr << "Point Idx? " << pointIdx << std::endl;
        clusters[pointIdx] = clusterIdx;
        const velodyne_pointcloud::PointOffsetIRL  point = cylindricProjection.getPointByIdx(pointIdx);
        int clusterSize = 1;
//        std::cerr << "probably not " << pointIdx << std::endl;
        for (int i = 0; i < edges[pointIdx].size(); ++i) {
            int nextPointIdx = edges[pointIdx][i];
            const velodyne_pointcloud::PointOffsetIRL& nextPoint = cylindricProjection.getPointByIdx(nextPointIdx);
            if ((nextPointIdx >= 0) && (clusters[nextPointIdx] == 0) && isEdge(point, nextPoint)) {
                clusterSize += dfs(nextPointIdx, clustersPtr, edges, cylindricProjection, clusterIdx);
            }
        }
        return clusterSize;
    }

    std::pair<std::vector<std::vector<int>>, int> clusterize(const NCylindricProjection::CylindricProjection& cylindricProjection) const {
        ROS_DEBUG("start clusterize");
        ROS_DEBUG("start cylindricProjectiionCreation");
        std::vector<std::vector<int>> edges = cylindricProjection.getEdges();

        std::vector<std::vector<int>> clusters = markGround(cylindricProjection); // 0 is not a ground, 1 is a ground
//
//        std::vector<std::vector<int>> visited(cylindricProjection.getScansNumber(), std::vector<int>(cylindricProjection.getScanLength(), 0));
//        int nextClusterIdx = 2;
//
//        std::cerr << "start dfs loop" << std::endl;
//        std::vector<int> clusterSizes(2, -1);
//        for (int i = 0; i < NCylindricProjection::N_COLUMNS; ++i) {
//            for (int j = 0; j < NCylindricProjection::N_RINGS; ++j) {
//                if (cylindricProjection.mask[i][j]) {
//                    int idx = NCylindricProjection::toIdx(i,j);
//                    if (clusters[idx] == 0) {
//                        clusterSizes.push_back(dfs(idx, &clusters, edges, cylindricProjection, nextClusterIdx));
//                        nextClusterIdx++;
//                    }
//                }
//            }
//        }
//        // fulter clusters
//        std::cerr << "end dfs loop" << std::endl;
//        std::vector<int> filteredClusterIndices{0,1};
//        int clustersNumber = 2;
//        for (int i = 2; i < clusterSizes.size(); ++i) {
//            if (clusterSizes[i] < config.min_cluster_size) {
//                filteredClusterIndices.push_back(0);
//            } else {
//                filteredClusterIndices.push_back(clustersNumber);
//                clustersNumber++;
//            }
//        }
//        ROS_INFO("Clusters number %d", clustersNumber);
//        for (int i = 0; i < clusters.size(); ++i) {
//            clusters[i] = filteredClusterIndices[clusters[i]];
//        }
//
//        ROS_DEBUG("end clusterize");
        return std::make_pair(clusters, 2); //clustersNumber);
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
//            point.b = 255;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGBA> colourClusters(const NCylindricProjection::CylindricProjection& cylindricProjection,
                                                     const std::vector<std::vector<int>>& clusters,
                                                     const std::vector<bool>& mask) const {
        ROS_DEBUG("start colourClusters");

        pcl::PointCloud<pcl::PointXYZRGBA> coloured_cloud;
//
//        ROS_DEBUG("test 1");
//        {
//            pcl::PointCloud<pcl::PointXYZRGBA> cloud;
//            cloud.header.frame_id = "velo_link";
//
//        }
//        ROS_DEBUG("test2");
//        {
//            pcl::PointCloud<pcl::PointXYZRGBA> cloud;
//
//            cloud.header.frame_id = "velo_link";
//            cloud.push_back(pcl::PointXYZRGBA());
//        }
//        ROS_DEBUG("sorry, it didn't work out");
//        coloured_cloud.header.frame_id = "velo_link";
//        for (int i = 0; i < NCylindricProjection::N_COLUMNS; ++i) {
//            for (int j = 0; j < NCylindricProjection::N_RINGS; ++j) {
//                if (cylindricProjection.mask[i][j]) {
//                    pcl::PointXYZRGBA point;
//                    point.x = cylindricProjection.cylindricProjection[i][j].x;
//                    point.y = cylindricProjection.cylindricProjection[i][j].y;
//                    point.z = cylindricProjection.cylindricProjection[i][j].z;
//                    colourPoint(point, clusters[i][j], mask);
//                    coloured_cloud.push_back(point);
//                }
//            }
//        }
        std::cerr << "Before initialization" << coloured_cloud.height << '\t' << coloured_cloud.width << '\t' << coloured_cloud.size() << std::endl;
//        coloured_cloud.height = 1;
//        coloured_cloud.width = coloured_cloud.size();

        ROS_DEBUG("end colourClusters");
        return coloured_cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBA> colourClusters(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud,
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