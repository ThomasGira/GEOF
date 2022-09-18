#include"sim.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace geoff{
namespace sim{

Sim::Sim(std::string map){
    this -> raw_map = geoff::viz::LoadImage(map);
    this -> raw_car = geoff::viz::LoadImage("../assets/car.jpg");
    geoff::common::Vector2d initial_pose = geoff::common::Vector2d(50,50,0);
    this -> car = geoff::sim::Car(initial_pose, raw_map);
};



}
}