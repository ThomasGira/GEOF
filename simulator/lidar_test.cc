#include"../utils/vectors.h"
#include"utils.h"
#include"lidar.h"
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
    geoff::sim::Lidar lidar = geoff::sim::Lidar(pose,raw_map, 30, 6.28, 100);
    while (true){
        lidar.check_lidar();
        lidar.draw_lidar();
    }
    return 0;
}