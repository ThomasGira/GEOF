#pragma once

#include"utils.h"
#include"visualizer.h"
#include"../utils/vectors.h"

namespace geoff{
namespace sim{

class Lidar{
    public:
        Lidar(geoff::common::Vector2d pose, cv::Mat map,  int num_beams, float fov, int range);
        Lidar();
        void update_pose(geoff::common::Vector2d pose);

        void draw_lidar();
        void check_lidar();
        std::vector<std::pair<int,int>> get_beam_points();
        cv::Mat get_beam_objs();
        std::vector<float> get_lidar_hits();
    private:
        int num_beams;
        float fov;
        int range;
        cv::Mat map;
        int accuracy;

        geoff::common::Vector2d pose;
        std::vector<std::pair<float,float>> beams;
        std::pair<float,float> calc_beam(float angle);
        bool check_hit(float angle, float length);
        void draw_beam(float angle, float length);
        
};
}}