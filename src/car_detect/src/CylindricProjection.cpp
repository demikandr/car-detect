//
// Created by demikandr on 5/1/18.
//

#include "CylindricProjection.h"

namespace NCylindricProjection {


        CylindricProjection::CylindricProjection(const pcl::PointCloud <velodyne_pointcloud::PointOffsetIRL> &cloud) :
                nPoints(cloud.size()),
                cylindricProjection(N_COLUMNS, std::vector<velodyne_pointcloud::PointOffsetIRL>(N_RINGS)),
                mask(N_COLUMNS, std::vector<int>(N_RINGS, 0)) {
            const auto getSlope = [](const velodyne_pointcloud::PointOffsetIRL &point) {
                return point.z / std::sqrt(point.x * point.x + point.y * point.y + 0.01);
            };
            const auto getDirection = [](const velodyne_pointcloud::PointOffsetIRL &point) {
                return std::atan2(point.x, point.y);
            };
            const auto getColumn = [&](const velodyne_pointcloud::PointOffsetIRL &point) {
                int result = std::floor((PI + getDirection(point) * 0.9999) / (2 * PI) * (N_COLUMNS));
//                std::cerr << getDirection(point) << '\t' << result << std::endl;
                assert(result >= 0);
                return result;
            };
            int ring = N_RINGS - 1;
            int startingColumn = getColumn(cloud[0]);
            for (int i = 0; i < cloud.size(); ++i) {
                int column = getColumn(cloud[i]);
                if ((i > 0) && (getColumn(cloud[i - 1]) > startingColumn) && (column <= startingColumn)) {
                    ring--;
                    assert(ring >=0);
                }

//            std::cerr << getSlope(cloud[i]) << '\t' << std::atan2(cloud[i].x, cloud[i].y) << '\t' <<  cloud[i].z << std::endl;
//            std::cerr << std::atan2(cloud[i].x, cloud[i].y) << std::endl;
                mask[column][ring] = 1;
                cylindricProjection[column][ring] = cloud[i];
            }
            assert(ring < 5);
//            assert(ring == 0);

        }

        std::vector<std::vector<int>> CylindricProjection::getEdges() const {
            std::vector<std::vector<int>> edges(nPoints);
            for (int i = 0; i < N_COLUMNS; ++i) {
                for (int j = 0; j < N_RINGS; ++j) {
                    if (mask[i][j] == 1) {
                        int pointIdx = toIdx(i, j);

                        {
                            int k = 1;
                            while (k < 10) {
                                int rightIdx = toIdx((i + k) % N_COLUMNS, j);
                                if (rightIdx >= 0) {
                                    edges[pointIdx].push_back(rightIdx);
                                    break;
                                }
                                k += 1;
                            }
                        }
                        for (int k = 1; k < 3; ++k) {
                            int lowerIdx = toIdx(i, (j + N_RINGS - k) % N_RINGS);
                            int upperIdx = toIdx(i, (j + k) % N_RINGS);
                            edges[pointIdx].push_back(lowerIdx);
                            edges[pointIdx].push_back(upperIdx);
                        }
                    }
                }
            }
            return edges;
        }

        int CylindricProjection::getScansNumber() const {
            return mask.size();
        }

        int CylindricProjection::getScanLength() const {
            assert(!mask.empty());
            return mask[0].size();
        }
}