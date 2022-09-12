#include"../utils/vectors.h"
#include"utils.h"
#include"visualizer.h"
#include<stdio.h>
#include<iostream>
#include<math.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv )
{
    cv::Mat raw_map = geoff::viz::LoadImage("../assets/map_1.jpg");
    cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);

    geoff::viz::CreateWindow("test");
    cv::Point pts[4] = {
        cv::Point(10, 10),
        cv::Point(200, 10),
        cv::Point(200,150),
        cv::Point(10,200)
    };
    cv::Mat map = raw_map.clone();
    bool collision = geoff::sim::check_collision(map,pts);
    if (collision){std::cout << "Colision" << std::endl;}
    else {std::cout << "We good chief" << std::endl;}
    geoff::viz::CreateWindow("test");

    while (true){
        geoff::viz::DisplayImage(raw_map,"test");

    }
    return 0;
}