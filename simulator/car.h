#pragma once

#include"lidar.h"
#include <opencv2/opencv.hpp>
#include"../utils/vectors.h"
#include"utils.h"
#include"visualizer.h"

namespace geoff{
namespace sim{

class Car{
    public:
        Car();
        Car(geoff::common::Vector2d pose, cv::Mat map);

        geoff::common::Vector2d get_pose(float ax, float at);
        void set_pose(geoff::common::Vector2d pose);
        void add_pose(geoff::common::Vector2d pose);
        bool check_collision(cv::Mat map);
        bool check_collision();
        cv::Mat draw(cv::Mat frame);
        geoff::common::Vector2d pose;

    private:
        float vel_x;
        float vel_theta;
        float length = 30;
        float width = 30;

        cv::Mat map;
        cv::Mat asset;
        std::chrono::steady_clock::time_point prev_time;
        std::chrono::steady_clock::time_point get_time();
        float get_dt();
        void update_velocity(float ax, float at, float dt);
        void update_pose(float dt);
};

}
}