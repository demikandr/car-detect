#include<vector>
#include "point_type.h"
#include<assert.h>
#include <pcl/point_cloud.h>

class CylindricProjection {
public:
    std::vector<std::vector<velodyne_pointcloud::PointOffsetIRL>> cylindricProjection;
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<int>> mask;
    int nPoints;

    CylindricProjection(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud):
            nPoints(cloud.size()),
            cylindricProjection(3000, std::vector<velodyne_pointcloud::PointOffsetIRL>(32)),
            indices(3000, std::vector<int>(32, -1)),
            mask(3000, std::vector<int>(32, 0))         {
        int currentWall = 0;
        for (int i = 0; i < cloud.size(); ++i) {
            if ((i > 0) && (cloud[i - 1].ring > cloud[i].ring)) {
                currentWall++;
            }
            indices[currentWall][cloud[i].ring] = i;
            mask[currentWall][cloud[i].ring] = 1;
            cylindricProjection[currentWall][cloud[i].ring] = cloud[i];
        }
    }

    std::vector<std::vector<int>> getEdges() const {
        std::vector<std::vector<int>> edges(nPoints);
        for (int i = 0; i < 3000; ++i) {
            for (int j = 0; j < 32; ++j) {
                if (mask[i][j] == 1) {
                    int pointIdx = indices[i][j];
                    for (int k = 1; k < 15; ++k) { // TODO(demikandr) make a parameter
                        int leftIdx = indices[(i + 3000 - k) % 3000][j];
                        int rightIdx = indices[(i + k) % 3000][j];
                        edges[pointIdx].push_back(leftIdx);
                        edges[pointIdx].push_back(rightIdx);
                    }
                    for (int k = 1; k < 3; ++k) {
                        int lowerIdx = indices[i][(j + 32 - k) % 32];
                        int upperIdx = indices[i][(j + k) % 32];
                        edges[pointIdx].push_back(lowerIdx);
                        edges[pointIdx].push_back(upperIdx);
                    }
                }
            }
        }
        return edges;
    }
    int getScansNumber() const {
        return mask.size();
    }
    int getScanLength() const {
        assert(!mask.empty());
        return mask[0].size();
    }

};