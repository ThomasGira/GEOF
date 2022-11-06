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

Lidar::Lidar(){
    map = geoff::viz::LoadImage("../assets/map_1.jpg");
    cv::resize(map, map, cv::Size(1000, 1000), cv::INTER_LINEAR);
    pose = geoff::common::Vector2d(100,100,0);
    this -> num_beams = 6;
    this -> fov = 6.28;
    this -> range = 300;
    this -> accuracy = 5;
};

Lidar::Lidar(cv::Mat map){
    this -> map = map;
    cv::resize(map, map, cv::Size(1000, 1000), cv::INTER_LINEAR);
    pose = geoff::common::Vector2d(100,100,0);
    this -> num_beams = 30;
    this -> fov = 6.28;
    this -> range = 400;
    this -> accuracy = 7;
};

void Lidar::update_pose(geoff::common::Vector2d pose){
    this -> pose = pose;
};
cv::Mat Lidar::get_beam_objs(){
    cv::Mat img(this->range*2, this->range*2,CV_8UC1,255);
    for (std::pair<float, float> beam : this -> beams) { 
        float dx = cos(beam.first) * beam.second;
        float dy = sin(beam.first) * beam.second;
        
        geoff::common::Vector2d p1 = geoff::common::Vector2d(this->range,this->range,0);
        geoff::common::Vector2d p2 = geoff::common::Vector2d(this->range+dx,this->range+dy,0);
        geoff::common::Vector2d p3 = geoff::common::Vector2d(this->range+dx+1.0, this->range+dy+1.0,0);
        geoff::common::Vector2d p4 = geoff::common::Vector2d(this->range+1, this->range+1,0);
        cv::Point pts[4] = {
            p1.to_cv_point(),
            p2.to_cv_point(),
            p3.to_cv_point(),
            p4.to_cv_point()
        };
        cv::fillConvexPoly( img, pts, 4, cv::Scalar(0,0,255) );
    }
    return img;

}
std::vector<std::pair<int,int>> Lidar::get_beam_points(){
    std::vector<std::pair<int,int>> points;
    for (std::pair<float, float> beam : this -> beams) { 
        int dx = (int) (cos(-pose.rho + beam.first) * beam.second);
        int dy = (int) (sin(-pose.rho + beam.first) * beam.second);
        std::pair<int,int> point(dx, dy);
        points.push_back(pose.point2world(point));
    }
    return points;

}
std::vector<float> Lidar::get_lidar_hits(){
    std::vector<float> points;
    for (std::pair<float, float> beam : this -> beams) { 
        float dx = cos(beam.first) * beam.second;
        float dy = sin(beam.first) * beam.second;
        points.push_back(dx/range);
        points.push_back(dy/range);
    }
    return points;

}

void Lidar::check_lidar(){
    std::vector<std::pair<float,float>> hits;
    float delta_angle = this -> fov /(this -> num_beams);
    for (int i = 0; i < this -> num_beams; i++){
        // std::cout << "Calculating beam " << std::to_string(i) << " with angle " << std::to_string(-pose.rho + delta_angle*i) << std::endl;
        std::pair<float,float> hit = calc_beam(-pose.rho + delta_angle*i);
        hits.push_back(hit);
    }

    beams = hits;
};

std::pair<float,float> Lidar::calc_beam(float angle){
    float length = this -> range;
    int iterations = 0;
    float max = length;
    float min = 0;
    while ( iterations < accuracy ){
        if (check_hit(angle,length)){
            max = length;
            length = (min + length) /2.0;
        } else {
            min = length;
            length = (max + length) /2.0;
        }
        iterations++;
    }
    // float dx = cos(angle) * length;
    // float dy = sin(angle) * length;
    // std::cout << "angle: " << std::to_string(angle) << " dx: " << dx << " dy: " << dy << std::endl;  
    // geoff::common::Vector2d p1 = geoff::common::Vector2d(pose.x,pose.y,0);
    // geoff::common::Vector2d p2 = geoff::common::Vector2d(pose.x + dx, pose.y + dy,0);
    // geoff::common::Vector2d p3 = geoff::common::Vector2d(pose.x + dx + 1.0, pose.y + dy + 1.0,0);
    // geoff::common::Vector2d p4 = geoff::common::Vector2d(pose.x + 1, pose.y + 1,0);
    // cv::Point pts[4] = {
    //     p1.to_cv_point(),
    //     p2.to_cv_point(),
    //     p3.to_cv_point(),
    //     p4.to_cv_point()
    // };
    // geoff::sim::draw_on(map,pts);
    return std::pair<float,float> {angle,length};
}
void Lidar::draw_lidar() {
    std::vector<std::vector<cv::Point>> point_list;
    for (std::pair<float, float> beam : beams) { 
        float dx = cos(-pose.rho + beam.first) * beam.second;
        float dy = sin(-pose.rho + beam.first) * beam.second;
        
        geoff::common::Vector2d p1 = geoff::common::Vector2d(pose.x,pose.y,0);
        geoff::common::Vector2d p2 = geoff::common::Vector2d(pose.x + dx,pose.y + dy,0);
        geoff::common::Vector2d p3 = geoff::common::Vector2d(pose.x + dx + 1.0, pose.y + dy + 1.0,0);
        geoff::common::Vector2d p4 = geoff::common::Vector2d(pose.x + 1, pose.y + 1,0);
        std::vector<cv::Point> pts;
        pts.push_back(p1.to_cv_point());
        pts.push_back(p2.to_cv_point());
        pts.push_back(p3.to_cv_point());
        pts.push_back(p4.to_cv_point());

        point_list.push_back(pts);
    }
    geoff::sim::draw_on(map,point_list);
}
void Lidar::draw_beam(float angle, float length){
    float dx = cos(-pose.rho + angle) * length;
    float dy = sin(-pose.rho + angle) * length;
        
    geoff::common::Vector2d p1 = geoff::common::Vector2d(pose.x,pose.y,0);
    geoff::common::Vector2d p2 = geoff::common::Vector2d(pose.x + dx,pose.y + dy,0);
    geoff::common::Vector2d p3 = geoff::common::Vector2d(pose.x + dx + 1.0, pose.y + dy + 1.0,0);
    geoff::common::Vector2d p4 = geoff::common::Vector2d(pose.x + 1, pose.y + 1,0);
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
        
    geoff::common::Vector2d p1 = geoff::common::Vector2d(pose.x,pose.y,0);
    geoff::common::Vector2d p2 = geoff::common::Vector2d(pose.x + dx,pose.y + dy,0);
    geoff::common::Vector2d p3 = geoff::common::Vector2d(pose.x + dx + 1.0, pose.y + dy + 1.0,0);
    geoff::common::Vector2d p4 = geoff::common::Vector2d(pose.x + 1, pose.y + 1,0);
    cv::Point pts[4] = {
        p1.to_cv_point(),
        p2.to_cv_point(),
        p3.to_cv_point(),
        p4.to_cv_point()
    };
    return geoff::sim::check_collision(map,pts);
}
}}