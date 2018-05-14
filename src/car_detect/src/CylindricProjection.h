#pragma once
#include<vector>
#include "point_type.h"
#include<assert.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <boost/math/constants/constants.hpp>

namespace NCylindricProjection {
    const int N_COLUMNS = 400; // 2150;
    const int N_RINGS = 64;
    const double PI = boost::math::constants::pi<double>();
    class CylindricProjection {
    public:
        std::vector<std::vector<velodyne_pointcloud::PointOffsetIRL>> cylindricProjection;
        std::vector<std::vector<int>> mask;
        int nPoints;

        CylindricProjection(const pcl::PointCloud <velodyne_pointcloud::PointOffsetIRL> &cloud);

        std::vector<std::vector<int>> getEdges() const;
        int getScansNumber() const;

        int getScanLength() const;


    };
    inline int toIdx(int x, int y) {
        return x * N_RINGS + y;
    }
}