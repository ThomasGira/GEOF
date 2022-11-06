#include"../utils/vectors.h"
#include"../feedforward/network.h"
#include"car.h"
#include "../utils/json.hpp"
using json = nlohmann::json;
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char** argv )
{
    // Get scenario.
    std::ifstream f("../simulator/setup.json");
    json setup = json::parse(f);

    std::string map_name = setup["map"];
    std::vector<float> spawn = setup["spawn"];
    std::vector<std::vector<float>> waypoints = setup["waypoints"];
    std::cout << "Map: " << map_name << std::endl;
    std::cout << "Spawn: " << std::to_string(spawn[0]) << ", " << std::to_string(spawn[1]) << ", " << std::to_string(spawn[2]) << std::endl;
    // Loop forever
    while (true) {
        // Save current config as historical file
        int curr_time = time(0);
        std::ifstream f("../feedforward/network.json");
        json data = json::parse(f);
        std::ofstream myfile;
        myfile.open ("../feedforward/historic/"+std::to_string(curr_time)+".json");
        myfile << data.dump(4);
        myfile.close();

        // Setup the best score
        json j;
        std::pair<int,json> best (0,j);
        
        // Loop throug multiple generations.
        for (int i =0; i< 5; i++) {
            // Create a new map
            cv::Mat raw_map = geoff::viz::LoadImage("../assets/"+map_name);
            cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);
            geoff::common::Vector2d pose = geoff::common::Vector2d(spawn[0],spawn[1],spawn[2]);
            geoff::viz::CreateWindow("test");
            
            // Create a new car and network.
            geoff::sim::Car car = geoff::sim::Car(pose,raw_map);
            geoff::ff::Network network = geoff::ff::Network();

            int index = 0;
            int score = 0;

            // Loop through untill maximum iterations or collision.
            while (!(car.check_collision(raw_map)) & index < 100000){
                index++;
                car.check_lidar();
                std::vector<float> lidar_hits= car.get_lidar_hits();
                network.determine_outputs(lidar_hits);
                std::vector<float> outputs = network.get_output();
                geoff::common::Vector2d add_pose = geoff::common::Vector2d(outputs[0],0,(outputs[1]-0.5)/3);
                car.add_pose(add_pose);
                // cv::Mat frame = car.draw();
                // geoff::viz::DisplayImage(frame,"test");

                if (waypoints.size() > 0){
                    if (car.distance_from_point(waypoints[0][0],waypoints[0][1]) < 10.0){
                        std::cout << "Hit waypoint: " << waypoints[0][0] << "," << waypoints[0][1] << std::endl;
                        score += 1000;
                        waypoints.erase(waypoints.begin());
                    }
                }
            }

            // Calculate performance
            cv::Mat traversed_area = car.get_traversed_area();
            score += car.get_score();
            json config = network.get_json();
            if (score > best.first){
                best.first = score;
                best.second = config;
            }
            std::cout <<"Score: " << score << std::endl;
        }
        // Update best config.
        if (best.first > data["score"]){
            best.second["score"] = best.first;
            std::cout << "Saving best Score: " << best.first << std::endl;
            myfile.open ("../feedforward/network.json");
            myfile << best.second.dump(4);
            myfile.close();
        } else {
            std::cout << "No performance improvements determined" << std::endl;
        }
    }
    
    return 0;
}