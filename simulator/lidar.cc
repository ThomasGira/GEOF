#include "lidar.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <chrono>

namespace geoff{
namespace sim{

Lidar::Lidar(geoff::common::Vector2d pose, cv::Mat map,  int num_beams, int fov, int range){
    this -> num_beams = num_beams;
    this -> fov = fov;
    this -> range = range;
    this -> map = map;
};

void Lidar::update_pose(geoff::common::Vector2d pose){
    this -> pose = pose;
};

std::vector<geoff::common::Vector2d> Lidar::check_lidar(){
    std::vector<geoff::common::Vector2d> hits;
    float max_angle = this -> fov/2.0;
    float delta_angle = this -> fov /(this -> num_beams-1);
    for (int i = 0; i < this -> num_beams; i++){
        geoff::common::Vector2d hit = check_hit(max_angle - delta_angle*i);
        hits.push_back(hit);
    }

    return hits;
};

geoff::common::Vector2d Lidar::check_hit(float angle){
    return this -> pose;
};

}}