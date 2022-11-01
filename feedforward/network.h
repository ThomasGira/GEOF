#pragma once

#include <vector>
#include "layer.h"
namespace geoff{
namespace ff{
/**
 * @brief A Feed Forward Neural Network.
*/
class Network{
    public:
        Network();
        /**
         * @brief A function to determine the outputs in a network..
         * @param inputs A vector of float inputs.
        */
        void determine_outputs(std::vector<float> inputs);

        /**
         * @brief A function to get the final output of a neural net.
         * @return A vector of floats representing the final output.
        */
        std::vector<float> get_output();

        /**
         * @brief A list of layers in the network.
        */
        std::vector<Layer> layers;

        /** @brief Calculates the current weights and biases that the neural net is using.
         * @return The json object
        */
        json get_json();
    private:
        /** 
         * @brief The number of layers in the network.
        */
        int num_layers;

        /**
         * @brief The range of randomness to apply to the input net schema.
        */ 
       float random;   
};
}}