#pragma once

#include"visualizer.h"
#include"car.h"
#include"../utils/vectors.h"

namespace geoff{
namespace sim{

class Sim{
    public:
        Sim(std::string map);

    private:
        cv::Mat raw_map;
        cv::Mat raw_car;
        geoff::sim::Car car;
};

}
}