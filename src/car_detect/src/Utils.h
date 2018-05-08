#pragma once
#include <pcl_conversions/pcl_conversions.h>

class Utils {
    const pcl::PointCloud <velodyne_pointcloud::PointOffsetIRL> &cloud;
    const int size;
public:
    Utils(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> &cloud): cloud(cloud), size(cloud.points.size()) {    }
    int safe_idx(int idx) const {
        return (idx + size) % size;
    }

    double l2(const int idx) const {
//        pcl::Vector3fMap current(cloud.points[safe_idx(idx)].getVector3fMap());
        velodyne_pointcloud::PointOffsetIRL point = cloud.points[safe_idx(idx)];

        double result = sqrt(point.x * point.x + point.y* point.y + point.z * point.z);// .getVector3fMap().lpNorm<2>();
        assert(result > 0);
        return result;
    }

    bool isLowestLevel(const int point_idx) const {
        return ( cloud.points[safe_idx(point_idx)].ring  < cloud.points[safe_idx(point_idx - 1)].ring);
    }

    bool isHighestLevel(const int point_idx) const {
        return isLowestLevel(point_idx + 1);
    }

    double oxyAngleCos(const int x_idx, const int y_idx) const {
        velodyne_pointcloud::PointOffsetIRL x = cloud.points[safe_idx(x_idx)];
        velodyne_pointcloud::PointOffsetIRL y = cloud.points[safe_idx(y_idx)];
        pcl::Vector3fMap vec1(x.getVector3fMap());
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return vec1.dot(vec2) / (vec1.lpNorm<2>()  * vec2.lpNorm<2>() + 0.01);
    }
    double distance(const int x_idx, const int y_idx) const {
        velodyne_pointcloud::PointOffsetIRL x = cloud.points[safe_idx(x_idx)];
        velodyne_pointcloud::PointOffsetIRL y = cloud.points[safe_idx(y_idx)];
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return vec2.lpNorm<2>();
    }
};

