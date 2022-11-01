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
        for (int i =0; i< 50; i++) {
            // Create a new map
            cv::Mat raw_map = geoff::viz::LoadImage("../assets/map_1.jpg");
            cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);
            geoff::common::Vector2d pose = geoff::common::Vector2d(150,300,-1.7);
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
                // cv::Mat frame = car.draw();
                // geoff::viz::DisplayImage(frame,"test");
            }

            // Calculate performance
            cv::Mat traversed_area = car.get_traversed_area();
            int score = car.get_score();
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