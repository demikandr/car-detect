//
// Created by demikandr on 5/1/18.
//

#include <ros/console.h>
#include "CylindricProjection.h"

namespace NCylindricProjection {


        CylindricProjection::CylindricProjection(const pcl::PointCloud <velodyne_pointcloud::PointOffsetIRL> &cloud) :
                cylindricProjection(N_COLUMNS, std::vector<velodyne_pointcloud::PointOffsetIRL>(N_RINGS)),
                mask(N_COLUMNS, std::vector<int>(N_RINGS, 0)) {
            ROS_DEBUG("start cylindric projection creation");
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
                assert(result < N_COLUMNS);
                return result;
            };
            int ring = 0;
            int startingColumn = getColumn(cloud[cloud.size() - 1]);
            for (int i = cloud.size() - 1; i >= 0; --i) {
                int column = getColumn(cloud[i]);
//                std::cerr << column << std::endl;
                if ((i + 1 < cloud.size()) && (getColumn(cloud[i + 1]) < startingColumn) && (column >= startingColumn)) {
                    ring++;
                    assert(ring < N_RINGS);
                }
                mask[column][ring] = 1;
                cylindricProjection[column][ring] = cloud[i], column;
            }
            std::cerr << "ring " << ring << std::endl;
            ROS_DEBUG("end cylindric projection creation");

        }

        std::vector<std::vector<int>> CylindricProjection::getEdges() const {
            std::vector<std::vector<int>> edges(N_COLUMNS * N_RINGS);
            for (int i = 0; i < N_COLUMNS; ++i) {
                for (int j = 0; j < N_RINGS; ++j) {
                    if (mask[i][j] == 1) {
                        int pointIdx = toIdx(i, j);

                        {
                            int k = 1;
                            while (k < 10) {
                                int columnIdx = (i + k) % N_COLUMNS;
                                if (mask[columnIdx][j]) {
                                    int rightIdx = toIdx(columnIdx, j);
                                    edges[pointIdx].push_back(rightIdx);
                                    break;
                                }
                                k += 1;
                            }
                        }

                        {
                            int k = 1;
                            while (k < 10) {
                                int columnIdx = (i + N_COLUMNS - k) % N_COLUMNS;
                                if (mask[columnIdx][j]) {
                                    int leftIdx = toIdx(columnIdx, j);
                                    edges[pointIdx].push_back(leftIdx);
                                    break;
                                }
                                k += 1;
                            }
                        }
                        for (int k = 1; k < 3; ++k) {
                            {
                                int rowIdx = (j + N_RINGS - k) % N_RINGS;
                                if (mask[i][rowIdx]) {
                                    int lowerIdx = toIdx(i, rowIdx);
                                    edges[pointIdx].push_back(lowerIdx);
                                }
                            }
                            {
                                int rowIdx = (j + k) % N_RINGS;
                                if (mask[i][rowIdx]) {
                                    int upperIdx = toIdx(i, rowIdx);
                                    edges[pointIdx].push_back(upperIdx);
                                }
                            }
                        }
                    }
                }
            }

            int n_edges = 0;
            for (int i = 0; i < edges.size(); ++i) {
                n_edges += edges[i].size();
            }
            std::cerr << n_edges << std::endl;
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