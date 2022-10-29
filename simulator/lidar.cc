#include "lidar.h"

#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <chrono>

namespace geoff{
namespace sim{

Lidar::Lidar(geoff::common::Vector2d pose, cv::Mat map,  int num_beams, float fov, int range){
    this -> pose = pose;
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
        geoff::common::Vector2d hit = calc_beam(delta_angle*i);
        hits.push_back(hit);
    }

    return hits;
};

geoff::common::Vector2d Lidar::calc_beam(float angle){
    float length = this -> range;
    while ( check_hit(angle,length) ){
        
        length -= 1;
        if (length < 0 ){
            break;
        }
    }
    draw_on(angle,length);
    if (length < 0) {length = 0; };
    float dx = cos(angle) * length;
    float dy = sin(angle) * length;
    std::cout << "Final beam length: " << length << std::endl;
    return geoff::common::Vector2d(dx,dy,0).robot2world(this -> pose);
}

void Lidar::disp_mask(float angle, float length){
    float dx = cos(angle) * length;
    float dy = sin(angle) * length;
    
    geoff::common::Vector2d p1 = geoff::common::Vector2d(0,0,0).robot2world(pose);
    geoff::common::Vector2d p2 = geoff::common::Vector2d(dx,dy,0).robot2world(pose);
    geoff::common::Vector2d p3 = geoff::common::Vector2d(dx+1.0, dy+1.0,0).robot2world(pose);
    geoff::common::Vector2d p4 = geoff::common::Vector2d(1, 1,0).robot2world(pose);
    cv::Point pts[4] = {
        p1.to_cv_point(),
        p2.to_cv_point(),
        p3.to_cv_point(),
        p4.to_cv_point()
    };
    geoff::sim::disp_mask(map,pts);
}

void Lidar::draw_on(float angle, float length){
    float dx = cos(angle) * length;
    float dy = sin(angle) * length;
    
    geoff::common::Vector2d p1 = geoff::common::Vector2d(0,0,0).robot2world(pose);
    geoff::common::Vector2d p2 = geoff::common::Vector2d(dx,dy,0).robot2world(pose);
    geoff::common::Vector2d p3 = geoff::common::Vector2d(dx+1.0, dy+1.0,0).robot2world(pose);
    geoff::common::Vector2d p4 = geoff::common::Vector2d(1, 1,0).robot2world(pose);
    cv::Point pts[4] = {
        p1.to_cv_point(),
        p2.to_cv_point(),
        p3.to_cv_point(),
        p4.to_cv_point()
    };
    geoff::sim::draw_on(map,pts);
}


bool Lidar::check_hit(float angle, float length){
    float dx = cos(angle) * length;
    float dy = sin(angle) * length;
    
    geoff::common::Vector2d p1 = geoff::common::Vector2d(0,0,0).robot2world(pose);
    geoff::common::Vector2d p2 = geoff::common::Vector2d(dx,dy,0).robot2world(pose);
    geoff::common::Vector2d p3 = geoff::common::Vector2d(dx+1.0, dy+1.0,0).robot2world(pose);
    geoff::common::Vector2d p4 = geoff::common::Vector2d(1, 1,0).robot2world(pose);
    cv::Point pts[4] = {
        p1.to_cv_point(),
        p2.to_cv_point(),
        p3.to_cv_point(),
        p4.to_cv_point()
    };
    return geoff::sim::check_collision(map,pts);
}
}}