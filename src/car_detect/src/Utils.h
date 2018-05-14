#pragma once
#include <pcl_conversions/pcl_conversions.h>

namespace Utils {
    double l2(const velodyne_pointcloud::PointOffsetIRL& point) {
        double result = sqrt(point.x * point.x + point.y* point.y + point.z * point.z);// .getVector3fMap().lpNorm<2>();
        assert(result > 0);
        return result;
    }

    double oxyAngleCos(velodyne_pointcloud::PointOffsetIRL x, velodyne_pointcloud::PointOffsetIRL y) {
        pcl::Vector3fMap vec1(x.getVector3fMap());
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return vec1.dot(vec2) / (vec1.lpNorm<2>()  * vec2.lpNorm<2>() + 0.01);
    }

    double distance(const velodyne_pointcloud::PointOffsetIRL& x, const velodyne_pointcloud::PointOffsetIRL& y) {
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return vec2.lpNorm<2>();
    }
};

