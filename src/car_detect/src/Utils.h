#pragma once
#include <pcl_conversions/pcl_conversions.h>

namespace Utils {
    double l2(const velodyne_pointcloud::PointOffsetIRL& point) {
        double result = sqrt(point.x * point.x + point.y* point.y + point.z * point.z);// .getVector3fMap().lpNorm<2>();
        assert(result > 0);
        return result;
    }

    double oxyAngleCos(velodyne_pointcloud::PointOffsetIRL x, velodyne_pointcloud::PointOffsetIRL y) {
        Eigen::Vector3f vec1(x.getVector3fMap());
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return vec1.dot(vec2) / (vec1.lpNorm<2>()  * vec2.lpNorm<2>() + 0.01);
    }

    double oxyAngle(velodyne_pointcloud::PointOffsetIRL x, velodyne_pointcloud::PointOffsetIRL y) {
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return std::atan2(vec2.z(), std::sqrt((vec2.x() * vec2.x()) + (vec2.y() * vec2.y())));
    }

    double variance(velodyne_pointcloud::PointOffsetIRL x, velodyne_pointcloud::PointOffsetIRL y,
                    velodyne_pointcloud::PointOffsetIRL z) {
        Eigen::Vector3f vec1(x.getVector3fMap());
        Eigen::Vector3f vec2(y.getVector3fMap());
        Eigen::Vector3f vec3(z.getVector3fMap());
//        return (vec1 + vec3 - vec2).lpNorm<2>() / (vec1 - vec3).lpNorm<2>();
        return (vec2 - vec1).cross(vec2-vec3).lpNorm<2>() / (vec3 - vec1).lpNorm<2>() / vec2.lpNorm<2>();
    }

    double distance(const velodyne_pointcloud::PointOffsetIRL& x, const velodyne_pointcloud::PointOffsetIRL& y) {
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return vec2.lpNorm<2>();
    }
};

