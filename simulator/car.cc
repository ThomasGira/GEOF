#include"car.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <chrono>

namespace geoff{
namespace sim{

Car::Car(geoff::common::Vector2d pose){
    this -> prev_time = get_time();
    this -> pose = pose;
    this -> vel_x = 0;
    this -> vel_theta = 0;
};

geoff::common::Vector2d Car::get_pose(float ax, float at){
    float dt = this -> get_dt();
    update_velocity(ax,at,dt);
    update_pose(dt);

    return this -> pose;
};

std::chrono::steady_clock::time_point Car::get_time(){
    return std::chrono::steady_clock::now();
};

float Car::get_dt(){
    std::chrono::steady_clock::time_point curr_time = get_time();
    std::chrono::duration<double> diff = curr_time - this -> prev_time;
    float dt = (float) diff.count();
    this -> prev_time = curr_time;
    return dt;
    
};

void Car::update_velocity(float ax, float at, float dt){
    this -> vel_x += ax * dt;
    this -> vel_theta += at * dt;
};

void Car::update_pose(float dt){
    float dx = this -> vel_x * dt;
    float dtheta = this -> vel_theta * dt;

    geoff::common::Vector2d rel_delta = geoff::common::Vector2d(dx,0,dtheta);
    geoff::common::Vector2d world_delta = rel_delta.robot2world(this -> pose);

    this -> pose.add_eq(world_delta);
};

}
}