#include"visualizer.h"
#include<iostream>
#include<math.h>

int main(int argc, char** argv )
{
    cv::Mat car = geoff::viz::LoadImage("../assets/car.jpg");
    cv::resize(car, car, cv::Size(30, 30), cv::INTER_LINEAR);
  
    cv::Mat raw_map = geoff::viz::LoadImage("../assets/map_1.jpg");
    cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);

    geoff::viz::CreateWindow("test");
    float rho = 0;
    while (true){
        float scale = sin(rho/100);
        int x = (int) 500 + scale*450;
        int y = (int) 500 + scale*450;
        rho+= .1;
        std::cout << "X: " << x << " Y: " << y << std::endl;
        cv::Mat map = raw_map.clone();
        cv::Mat disp = geoff::viz::PlaceObject(map,car,x,y,rho);
        geoff::viz::DisplayImage(disp,"test");
    }
    return 0;
}