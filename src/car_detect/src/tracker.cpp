#include "point_type.h"
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<iostream>
#include<string>
#include<algorithm>
//#include <pcl/point_cloud.h>
//#include <pcl/impl/point_types.hpp>


ros::Publisher pub;
std::string version="0.004";

class Utils {
    const pcl::PointCloud <pcl::PointXYZRGB> &cloud;
    const int size;
public:
    Utils(const pcl::PointCloud<pcl::PointXYZRGB> &cloud): cloud(cloud), size(cloud.points.size()) {    }
    int safe_idx(int idx) const {
        return (idx + size) % size;
    }

    double l2(const int idx) const {
//        pcl::Vector3fMap current(cloud.points[safe_idx(idx)].getVector3fMap());
        pcl::PointXYZRGB point = cloud.points[safe_idx(idx)];

        double result = sqrt(point.x * point.x + point.y* point.y + point.z * point.z);// .getVector3fMap().lpNorm<2>();
        assert(result > 0);
        return result;
    }

    bool isLowestLevel(const int point_idx) const {
        double relative_distance = (l2(point_idx - 1) - l2(point_idx)) ; //// std::max( l2(point_idx - 1), l2(point_idx)) );
//        assert(relative_distance 52< 1);
        if (relative_distance> 4) {
            std::cerr <<'\n' << point_idx <<  '\t' << cloud.points[point_idx] << std::endl;
            std::cerr << safe_idx(point_idx-1) << '\t' << cloud.points[safe_idx(point_idx-1)] << std::endl;
        }
        return ( relative_distance> 50);
//    or (cloud.points[safe_idx(point_idx - 1)].z - cloud.points[safe_idx(point_idx)].z > 1);
    }

    bool isHighestLevel(const int point_idx) const {
        return isLowestLevel(point_idx + 1);
    }

    double oxyAngleCos(const int x_idx, const int y_idx) const {
        pcl::PointXYZRGB x = cloud.points[x_idx];
        pcl::PointXYZRGB y = cloud.points[y_idx];
        pcl::Vector3fMap vec1(x.getVector3fMap());
        Eigen::Vector3f vec2(y.getVector3fMap() - x.getVector3fMap());
        return vec1.dot(vec2) / (vec1.lpNorm<2>()  * vec2.lpNorm<2>() + 0.01);
    }
};

void clusterize(pcl::PointCloud<pcl::PointXYZRGB>* cloud) {
    std::vector<std::vector<int>> edges(cloud->size());
    std::vector<bool> groundMask(cloud->size());
    Utils utils(*cloud);
    for (int i = 0; i < cloud->size(); ++i) {
//        std::cerr << i << std::endl;
//        if (!utils.isHighestLevel(i) && (abs(1 - utils.oxyAngleCos(i, i+1)) < 0.1 ) && ((utils.isLowestLevel(i))  or groundMask[utils.safe_idx(i-1)])) {
        if (((utils.isLowestLevel(i)))) { //}} && !groundMask[utils.safe_idx(i-1)]) {
            groundMask[i] = true;
            cloud->at(i).b = 128;
        } else {
            cloud->at(i).g = 128 ;
        }
    }
}

int ii = 0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  // Create a container for the data.

    pcl::PointCloud<velodyne_pointcloud::PointOffsetIRL> cloud;
    pcl::fromROSMsg(*input, cloud);
    //    pcl::
//        clusterize(&cloud);
//        std::cerr << cloud.points[15].x <<  '\t' << cloud.points[15].y << '\t' << cloud.points[15].z << '\t' << cloud.points.size() << std::endl;
//        std::cerr << cloud.points[30015].x <<  '\t' << cloud.points[30015].y << '\t' << cloud.points[30015].z << '\t' << cloud.points.size() << std::endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    pub.publish(output);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "tracker_fast");
  ros::NodeHandle nh;
    std::cerr << "version" << version << std::endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1000);

  // Spin
  ros::spin ();
}
