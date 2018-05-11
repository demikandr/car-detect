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

class BBox {
public:
    float xMin, xMax;
    float yMin, yMax;
    float zMin, zMax;
    bool initialized = false;
    BBox init(const float x, const float y, const float z) {
        xMin = x;
        xMax = x;
        yMin = y;
        yMax = y;
        zMin = z;
        zMax = z;
        initialized = true;
    }
    BBox(){}
    BBox(const float xMin, const float yMin, const float zMin,
        const float xMax, const float yMax, const float zMax):
            xMin(xMin), xMax(xMax), yMin(yMin), yMax(yMax), zMin(zMin), zMax(zMax), initialized(true) {}
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
        if (!initialized) {
             init(point.z, point.y, point.z);
        } else {
            float x = point.x,
                    y = point.y,
                    z = point.z;
            xMin = std::min(xMin, x);
            yMin = std::min(yMin, y);
            zMin = std::min(zMin, z);
            xMax = std::max(xMax, x);
            yMax = std::max(yMax, y);
            zMax = std::max(zMax, z);
        }
    }


    car_detect::TrackedObject getTrackedObject() const {
        car_detect::TrackedObject trackedObject;

        trackedObject.pose.pose.position.x = (xMin + xMax) / 2;
        trackedObject.pose.pose.position.y = (yMin + yMax) / 2;
        trackedObject.pose.pose.position.z = (zMin + zMax) / 2;
        trackedObject.dims.dimensions.x = (xMax - xMin);
        trackedObject.dims.dimensions.y = (yMax - yMin);
        trackedObject.dims.dimensions.z = (zMax - zMin);
        return trackedObject;
    }

};

class Detections {
public:
    std::vector<BBox> detections;
    Detections(const pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL>& cloud, const std::vector<int>& pointToCluster, const int clustersNumber) {
        detections.clear();
        detections.resize(clustersNumber - 2);
        assert(cloud.size() == pointToCluster.size());
        for (int i = 0; i < cloud.size(); ++i) {
            if (pointToCluster[i] > 1) {
                detections[pointToCluster[i] - 2].add(cloud[i]);
            }
        }
        for (const auto detection: detections) {
            assert(detection.initialized);
            assert(detection.xMax > detection.xMin + 1e-2);
            assert(detection.yMax >= detection.yMin);
            assert(detection.yMax > detection.yMin+ 1e-2);
            assert(detection.zMax >= detection.zMin);
            assert(detection.zMax > detection.zMin+ 1e-2);
        }
    }
    boost::optional<int> findClosestDetectionIdxO(const BBox&  detection, const float threshold=0.5) const {
        assert(threshold > 0);
        float maxIOU = threshold; // intersection over union
        int closestDetectionIdx = -1;
        for (int i = 0; i < detections.size(); ++i) {
            float IOU = detection.getIOU(detections[i]);
            if (IOU > maxIOU) {

        for (const auto detection: detections) {
            assert(detection.initialized);
            assert(detection.xMax > detection.xMin);
            assert(detection.yMax >= detection.yMin);
            assert(detection.yMax > detection.yMin);
            assert(detection.zMax >= detection.zMin);
            assert(detection.zMax > detection.zMin);
        } maxIOU = IOU;
                closestDetectionIdx = i;
            }
        }
        if (closestDetectionIdx) {
            return closestDetectionIdx;
        } else {
            return {};
        }
    }

    boost::optional<BBox> findClosestDetectionO(const BBox& detection, const float threshold=0.5) const {
        boost::optional<int> closestDetectionIdx = findClosestDetectionIdxO(detection, threshold);
        if (closestDetectionIdx) {
            return detections[*closestDetectionIdx];
        } else {
            return {};
        }
    }
    car_detect::TrackedObjects getTrackedObjects() const {
        car_detect::TrackedObjects trackedObjects;
        for (const BBox& bbox: detections) {
            trackedObjects.trackedObjects.push_back(bbox.getTrackedObject());
        }
        return trackedObjects;
    }
};


#endif //PROJECT_DETECTIONS_H
