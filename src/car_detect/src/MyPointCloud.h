//
// Created by demikandr on 6/24/18.
//

#ifndef PROJECT_MYPOINTCLOUD_H
#define PROJECT_MYPOINTCLOUD_H


#include <pcl/point_types.h>

#typedef MyPoint PointXYZI

class MyPointCloud: public TMatrix<MyPoint> {
private:
    CloudMask mask;
public:
    MyPointCloud initFromPCL(const sensor_msgs::PointCloud2ConstPtr& input);
};


#endif //PROJECT_MYPOINTCLOUD_H
