#include"../utils/vectors.h"
#include"../feedforward/network.h"
#include"car.h"
#include "../utils/json.hpp"
using json = nlohmann::json;
#include <time.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv )
{
    // Get scenario.
    std::ifstream f("../simulator/setup.json");
    json setup = json::parse(f);

    std::string map_name = setup["map"];
    std::vector<float> spawn = setup["spawn"];
    std::cout << "Map: " << map_name << std::endl;
    std::cout << "Spawn: " << std::to_string(spawn[0]) << ", " << std::to_string(spawn[1]) << ", " << std::to_string(spawn[2]) << std::endl;
    // Loop forever
    while (true) {
        // Create a new map
        cv::Mat raw_map = geoff::viz::LoadImage("../assets/"+map_name);
        std::cout<< "lodaded" << std::endl;
        cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);
        geoff::common::Vector2d pose = geoff::common::Vector2d(spawn[0],spawn[1],spawn[2]);
        geoff::viz::CreateWindow("test");
        
        // Create a new car and network.
        geoff::sim::Car car = geoff::sim::Car(pose,raw_map);
        geoff::ff::Network network = geoff::ff::Network();

        int index = 0;

        // Loop through untill maximum iterations or collision.
        while (!(car.check_collision(raw_map))){
            index++;
            car.check_lidar();
            std::vector<float> lidar_hits= car.get_lidar_hits();
            network.determine_outputs(lidar_hits);
            std::vector<float> outputs = network.get_output();
            geoff::common::Vector2d add_pose = geoff::common::Vector2d(outputs[0],0,(outputs[1]-0.5)/3);
            car.add_pose(add_pose);
            cv::Mat frame = car.draw();
            geoff::viz::DisplayImage(frame,"test");
        }
    }
    
    return 0;
}