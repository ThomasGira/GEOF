#pragma once

#include"lidar.h"
#include"../utils/vectors.h"

namespace geoff{
namespace sim{

class Car{
    public:
        Car();
        Car(geoff::common::Vector2d pose);

        geoff::common::Vector2d get_pose(float ax, float at);
    private:
        geoff::common::Vector2d pose;
        
        float vel_x;
        float vel_theta;

        std::chrono::steady_clock::time_point prev_time;

        std::chrono::steady_clock::time_point get_time();

        float get_dt();

        void update_velocity(float ax, float at, float dt);

        void update_pose(float dt);

};

}
}