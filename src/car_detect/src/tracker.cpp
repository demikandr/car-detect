#include <ros/ros.h>
#include "MainLoop.h"


int main (int argc, char** argv) {
    ros::init (argc, argv, MainLoop::CNodeName);
    MainLoop loop();
    loop.spin();
}
