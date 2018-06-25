#pragma once
#include <dynamic_reconfigure/server.h>
#include <car_detect/generalConfig.h>
#include <boost/phoenix/bind/bind_member_function.hpp>
//
// Created by demikandr on 5/4/18.
//
class ConfigWrapper: public  car_detect::generalConfig {
    private:

        dynamic_reconfigure::Server<car_detect::generalConfig> server;
        dynamic_reconfigure::Server<car_detect::generalConfig>::CallbackType f;

        void callback(car_detect::generalConfig &config, uint32_t level) {
            car_detect::generalConfig::operator=(config);
            ROS_INFO("Reconfigure Request: %d %f", config.min_cluster_size, config.alpha);
        }

    public:
        ConfigWrapper() {
            f = boost::bind(&ConfigWrapper::callback, this, _1, _2);
            server.setCallback(f);
        }
};