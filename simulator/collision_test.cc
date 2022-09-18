#include"../utils/vectors.h"
#include"utils.h"
#include"car.h"
#include"visualizer.h"
#include<stdio.h>
#include<iostream>
#include<math.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv )
{
    cv::Mat raw_map = geoff::viz::LoadImage("../assets/map_1.jpg");
    cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);
    geoff::common::Vector2d pose = geoff::common::Vector2d(100,100,0);
    geoff::viz::CreateWindow("test");
    geoff::sim::Car car = geoff::sim::Car(pose,raw_map);
    float rho = 0;
    while (true){
        float scale = sin(rho/100);
        int x = (int) 500 + scale*450;
        int y = (int) 500 + scale*450;
        rho+= .1;
        geoff::common::Vector2d pose = geoff::common::Vector2d(x,y,rho);
        car.set_pose(pose);
        cv::Mat map = raw_map.clone();
        bool collision = car.check_collision(raw_map);
        if (collision){std::cout << "Colision" << std::endl;}
        else {std::cout << "We good chief" << std::endl;}
    }
    return 0;
}