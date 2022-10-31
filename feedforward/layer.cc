#include "layer.h"
#include<iostream>
#include <assert.h>

namespace geoff{
namespace ff{

Layer::Layer(){
    this -> num_nodes = 12;
    for (int i=0; i<num_nodes; i++){
        nodes.push_back(Node());
    }
};

Layer::Layer(json layer_config, float random){
    this -> num_nodes = layer_config.size();
    for (int i=0; i<num_nodes; i++){
        std::vector<float> weights = layer_config[i]["weights"];
        float bias = layer_config[i]["bias"];
        nodes.push_back(Node(weights,bias,random));
    }
};

void Layer::determine_output(std::vector<float> inputs){
    for (int i=0; i<num_nodes; i++){
        nodes[i].determine_output(inputs);
    }
}

void Layer::determine_output(std::vector<Node> inputs){
    for (int i=0; i<num_nodes; i++){
        nodes[i].determine_output(inputs);
    }
}

std::vector<float> Layer::get_outputs(){
    std::vector<float> outputs;
    for (Node node : nodes){
        outputs.push_back(node.output);
    }
    return outputs;
}
}}