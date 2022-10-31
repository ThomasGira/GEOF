#include "node.h"
#include <math.h>
#include <assert.h>
#include <iostream>

namespace geoff{
namespace ff{

Node::Node(){
    this -> length = 12;
    float percent = (float) rand() / (float) RAND_MAX;
    float shifted = percent -0.5;
    float normalized = shifted *20.0;
    this -> bias = normalized;
    this -> output = 0;
    for ( int i=0; i< length; i++){
        percent = (float) rand() / (float) RAND_MAX;
        shifted = percent -0.5;
        normalized = shifted *2.0;
        weights.push_back(normalized);
    }
};

Node::Node(std::vector<float> weights,float bias, float random){
    float percent = (float) rand() / (float) RAND_MAX;
    float shifted = percent -0.5;
    float normalized = shifted *random;
    this -> length = weights.size();
    this -> bias = bias + normalized;
    this -> output = 0;
    for ( int i=0; i< length; i++){
        percent = (float) rand() / (float) RAND_MAX;
        shifted = percent -0.5;
        normalized = shifted * random;
        // std::cout << "random: " << normalized << std::endl;
        this -> weights.push_back(weights[i] + normalized);
    }
};

void Node::determine_output(std::vector<float> input){
    // Check inputs are valid.
    assert(input.size() == length);
    // Apply Bias
    float sum = bias;
    // Sum inputs.
    for (int i=0; i< length; i++){
        sum += input[i]*weights[i];
    }
    // Apply sigmoid function.
    output = sigmoid(sum);
    // std::cout << "sum: " << sum << " output: " << output << std::endl;
}

void Node::determine_output(std::vector<Node> input){
    // Check inputs are valid.
    assert(input.size() == length);
    // Apply Bias
    float sum = bias;
    // Sum inputs.
    for (int i=0; i< length; i++){
        sum += input[i].output*weights[i];
    }
    // Apply sigmoid function.
    output = sigmoid(sum);
}

float Node::sigmoid(float input){
    return 1.0 / ( 1 + exp(-1.0*input) );
};
}}