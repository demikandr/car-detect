//
// Created by demikandr on 6/24/18.
//

#include "MyPointCloud.h"

MyPointCloud MyPointCloud::initFromPCL(const sensor_msgs::PointCloud2ConstPtr& input, int numColumns, int numRows):
    TMatrix<MyPoint>(numColumns, numRows)
{
    pcl::PointCloud<MyPoint> cloud;
    pcl::fromROSMsg(*input, cloud);

    const auto getSlope = [](const MyPoint &point) {
        return point.z / std::sqrt(point.x * point.x + point.y * point.y + 0.01);
    };

    const auto getDirection = [](const MyPoint &point) {
        return std::atan2(point.x, point.y);
    };

    const auto getColumn = [&](const MyPoint &point) {
        int result = std::floor((PI + getDirection(point) * 0.9999) / (2 * PI) * (N_COLUMNS));
        assert(result >= 0);
        assert(result < N_COLUMNS);
        return result;
    };

    int ring = 0;
    int startingColumn = getColumn(cloud[cloud.size() - 1]);

    for (int i = cloud.size() - 1; i >= 0; --i) {
        int column = getColumn(cloud[i]);
        if ((i + 1 < cloud.size()) && (getColumn(cloud[i + 1]) < startingColumn) && (column >= startingColumn)) {
            ring++;
            assert(ring < N_RINGS);
        }
        mask[column][ring] = 1;
        cylindricProjection[column][ring] = cloud[i], column;
    }
}