#include "network.h"
#include<iostream>
#include "../utils/json.hpp"
using json = nlohmann::json;
#include <fstream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>

namespace geoff{
namespace ff{

Network::Network(){
    srand(time(0));
    std::ifstream f("../feedforward/network.json");
    json data = json::parse(f);
    json layers = data["layers"];
    float random = data["random"];
    this -> num_layers = 0;
    for (json layer_config : layers){
        num_layers++;
        this->layers.push_back(Layer(layer_config,random));
    }
};

void Network::determine_outputs(std::vector<float> inputs){
    layers[0].determine_output(inputs);
    for (int i=1; i < num_layers; i++){
        // std::cout << "layer: " << i << std::endl;
        layers[i].determine_output(layers[i-1].get_outputs());
    }

}


std::vector<float> Network::get_output(){
    return layers.back().get_outputs();
}

}}