//
// Created by demikandr on 6/23/18.
//

#ifndef PROJECT_MAINLOOP_H
#define PROJECT_MAINLOOP_H

#include <sensor_msgs/PointCloud2.h>
#include <ros/callback_queue.h>


class MainLoop {
private:
    ros::NodeHandle nh;
    ConfigWrapper config;
    ros::Subscriber sub;
    ros::Publisher pub, bboxPub, velocityPub;
    void callback(const sensor_msgs::PointCloud2ConstPtr& input);
public:
    static const std::string CNodeName = "tracker_fast";
    MainLoop();
    void spin() {
        ros::spin();
    }
};


#endif //PROJECT_MAINLOOP_H
