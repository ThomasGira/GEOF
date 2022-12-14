#pragma once

#include"lidar.h"
#include <opencv2/opencv.hpp>
#include"../utils/vectors.h"
#include"utils.h"
#include"visualizer.h"
#include"lidar.h"

namespace geoff{
namespace sim{

class Car{
    public:
        Car();
        Car(geoff::common::Vector2d initial_pose, cv::Mat base_map);

        geoff::common::Vector2d get_pose(float ax, float at);
        void set_pose(geoff::common::Vector2d pose);
        /** 
         * Adds a pose in robot coordinates
        */
        void add_pose(geoff::common::Vector2d pose);
        bool check_collision(cv::Mat map);
        bool check_collision();
        cv::Mat draw(cv::Mat frame);
        cv::Mat draw();
        geoff::common::Vector2d pose;
        void check_lidar();
        void draw_lidar();
        std::vector<float> get_lidar_hits();
        cv::Mat get_traversed_area();
        int get_score();
        float distance_from_point(float x, float y);
    private:
        float vel_x;
        float vel_theta;
        float length = 30;
        float width = 30;
        geoff::sim::Lidar lidar;
        cv::Mat map;
        cv::Mat asset;
        cv::Mat traversed_area;
        std::chrono::steady_clock::time_point prev_time;
        std::chrono::steady_clock::time_point get_time();
        float get_dt();
        void update_velocity(float ax, float at, float dt);
        void update_pose(float dt);
};

}
}