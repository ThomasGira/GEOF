#include"../utils/vectors.h"
#include"../navigation/keyboard.h"
#include"utils.h"
#include"car.h"
#include"visualizer.h"
#include<stdio.h>
#include<math.h>

int main(int argc, char** argv )
{
    cv::Mat raw_map = geoff::viz::LoadImage("../assets/map_1.jpg");
    cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);
    geoff::common::Vector2d pose = geoff::common::Vector2d(120,130,1);
    geoff::viz::CreateWindow("test");
    geoff::sim::Car car = geoff::sim::Car(pose,raw_map);
    float rho = 0;
    geoff::key::init_key();
    while (true){
        int key = geoff::key::get_key();
        geoff::common::Vector2d add_pose;
        switch (key){
            case 0:
                add_pose = geoff::common::Vector2d(1,0,0);
                break;
            case 1:
                add_pose = geoff::common::Vector2d(0,0,-.1);
                break;
            case 2:
                add_pose = geoff::common::Vector2d(-1,0,0);
                break;
            case 3:
                add_pose = geoff::common::Vector2d(0,0,.1);
                break;
            case 4:
                add_pose = geoff::common::Vector2d(0,0,0);
                break;
            default:
                add_pose = geoff::common::Vector2d(0,0,0);
                break;
        }
        car.add_pose(add_pose);
        bool collision = car.check_collision(raw_map);
        car.check_lidar();
        cv::Mat frame = car.draw();
        geoff::viz::DisplayImage(frame,"test");
    }
    return 0;
}