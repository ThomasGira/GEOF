#pragma once

#include"utils.h"
#include"visualizer.h"
#include"../utils/vectors.h"

namespace geoff{
namespace sim{

class Lidar{
    public:
        Lidar(geoff::common::Vector2d pose, cv::Mat map,  int num_beams, float fov, int range);

        void update_pose(geoff::common::Vector2d pose);

        std::vector<geoff::common::Vector2d> check_lidar();
    private:
        int num_beams;
        float fov;
        int range;
        cv::Mat map;

        geoff::common::Vector2d pose;

        geoff::common::Vector2d calc_beam(float angle);
        bool check_hit(float angle, float length);
        void disp_mask(float angle, float length);
        void draw_on(float angle, float length);
        
};
}}