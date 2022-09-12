#pragma once

#include"visualizer.h"
#include"../utils/vectors.h"

namespace geoff{
namespace sim{

class Lidar{
    public:
        Lidar(geoff::common::Vector2d pose, cv::Mat map,  int num_beams, int fov, int range);

        void update_pose(geoff::common::Vector2d pose);

        std::vector<geoff::common::Vector2d> check_lidar();
    private:
        int num_beams;
        int fov;
        int range;
        cv::Mat map;

        geoff::common::Vector2d pose;

        geoff::common::Vector2d check_hit(float angle);
};
}}