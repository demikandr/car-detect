//
// Created by demikandr on 5/2/18.
//

#ifndef PROJECT_DETECTIONS_H
#define PROJECT_DETECTIONS_H

#include <vector>
#include <assert.h>
#include <pcl/point_cloud.h>
#include "point_type.h"
#include <car_detect/TrackedObject.h>
#include <car_detect/TrackedObjects.h>
#include <boost/optional.hpp>
#include "CylindricProjection.h"

class BBox {
public:
    float xMin, xMax, xCenter;
    float yMin, yMax, yCenter;
    float zMin, zMax, zCenter;
    float speed = 0;
    bool markeredPredok = false;
    int pointsNumber = 0;
    bool initialized = false;
    BBox init(const float x, const float y, const float z) {
        xMin = x;
        xMax = x;
        xCenter = x;
        yMin = y;
        yMax = y;
        yCenter = y;
        zMin = z;
        zMax = z;
        zCenter = z;
        initialized = true;
        pointsNumber = 1;
    }
    BBox() = default;
    BBox(const float xMin, const float yMin, const float zMin,
        const float xMax, const float yMax, const float zMax):
            xMin(xMin), xMax(xMax), yMin(yMin), yMax(yMax), zMin(zMin), zMax(zMax),
            pointsNumber(-1), initialized(true) {}
    BBox intersect(const BBox& other) const {
        assert(this->intersects(other));

        return BBox(std::max(xMin, other.xMin), std::max(yMin, other.yMin), std::max(zMin, other.zMin),
            std::min(xMax, other.xMax), std::min(yMax, other.yMax), std::min(zMax, other.zMax));
    }

    bool intersects(const BBox& detection2) const {
        return ((this->xMax > detection2.xMin)
                and (this->yMax > detection2.yMin)
                and (this->zMax > detection2.zMin)
                and (detection2.xMax > this->xMin)
                and (detection2.yMax > this->yMin)
                and (detection2.zMax > this->zMin));
    }

    float volume() const {
        return (xMax - xMin) * (yMax - yMin) * (zMax - zMin);
    }

    float getIOU(const BBox& other) const {
        if (this->intersects(other)) {
            float intersectionVolume = this->intersect(other).volume();
            return intersectionVolume / (this->volume() + other.volume() - intersectionVolume);
        } else {
            return 0.;
        }
    }
    void add(const velodyne_pointcloud::PointOffsetIRL& point) {
        assert (point.x < 10000);
        assert (point.y < 10000);
        assert (point.z < 10000);
        assert (point.x > -10000);
        assert (point.y > -10000);
        assert (point.z > -10000);
//        assert (point.x != 0);
//        assert(point.y != 0);
//        assert(point.z != 0);
        if (!initialized) {
             init(point.x, point.y, point.z);
        } else {
            pointsNumber++;
            float x = point.x,
                    y = point.y,
                    z = point.z;
            xMin = std::min(xMin, x);
            yMin = std::min(yMin, y);
            zMin = std::min(zMin, z);
            xMax = std::max(xMax, x);
            yMax = std::max(yMax, y);
            zMax = std::max(zMax, z);
            xCenter = xCenter * (pointsNumber - 1) / pointsNumber + x / pointsNumber;
            yCenter = yCenter * (pointsNumber - 1) / pointsNumber + y / pointsNumber;
            zCenter = zCenter * (pointsNumber - 1) / pointsNumber + z / pointsNumber;
        }
    }


    car_detect::TrackedObject getTrackedObject() const {
        car_detect::TrackedObject trackedObject;

        trackedObject.pose.pose.position.x = xMin;
        trackedObject.pose.pose.position.y = yMin;
        trackedObject.pose.pose.position.z = zMin;
        trackedObject.dims.dimensions.x = (xMax - xMin);
        trackedObject.dims.dimensions.y = (yMax - yMin);
        trackedObject.dims.dimensions.z = (zMax - zMin);
        return trackedObject;
    }

};

class Detections {
public:
    std::vector<BBox> detections;
    Detections(const NCylindricProjection::CylindricProjection& cloud, const std::vector<std::vector<int>>& pointToCluster, const int clustersNumber) {
        detections.clear();
        detections.resize(clustersNumber - 2);
        for (int i = 0; i < pointToCluster.size(); ++i) {
            for (int j = 0; j < pointToCluster[0].size(); ++j) {
                if (pointToCluster[i][j] > 1) {
                    assert(cloud.mask[i][j]);
                    assert(detections.size() + 2 > pointToCluster[i][j]);
                    detections[pointToCluster[i][j] - 2].add(cloud.cylindricProjection[i][j]);
                }
            }
        }
        for (const auto detection: detections) {
            assert(detection.initialized);
            assert(detection.xMax >= detection.xMin);
//            assert(detection.xMax > detection.xMin + 1e-2);
            assert(detection.yMax >= detection.yMin);
//            assert(detection.yMax > detection.yMin+ 1e-2);
            assert(detection.zMax >= detection.zMin);
//            assert(detection.zMax > detection.zMin+ 1e-2);
//            assert(detection.xMax != 0);
//            assert(detection.yMax != 0);
//            assert(detection.zMax != 0);
//            assert(detection.xMax != 0);
//            assert(detection.yMax != 0);
//            assert(detection.zMax != 0);
        }
    }
    boost::optional<int> findClosestDetectionIdxO(const BBox&  detection, const float threshold=0.5) const {
        assert(threshold > 0);
        float maxIOU = threshold; // intersection over union
        int closestDetectionIdx = -1;
        for (int i = 0; i < detections.size(); ++i) {
            float IOU = detection.getIOU(detections[i]);
            if (IOU > maxIOU) {
                 maxIOU = IOU;
                closestDetectionIdx = i;
            }
        }
        if (closestDetectionIdx>=0) {
            return closestDetectionIdx;
        } else {
            return boost::none;
        }
    }

    boost::optional<BBox> findClosestDetectionO(const BBox& detection, const float threshold=0.5) const {
        boost::optional<int> closestDetectionIdx = findClosestDetectionIdxO(detection, threshold);
        if (closestDetectionIdx) {
            return detections[closestDetectionIdx.get()];
        } else {
            return boost::none;
        }
    }
    car_detect::TrackedObjects getTrackedObjects(const std::vector<bool> mask) const {
        car_detect::TrackedObjects trackedObjects;
        for (int i = 0; i < detections.size(); ++i) {
            if (mask[i+2]) {
                trackedObjects.trackedObjects.push_back(detections[i].getTrackedObject());
            }
        }
        return trackedObjects;
    }
};


#endif //PROJECT_DETECTIONS_H
