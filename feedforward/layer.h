#pragma once

#include <vector>
#include "node.h"
#include "../utils/json.hpp"
using json = nlohmann::json;

namespace geoff{
namespace ff{
/**
 * @brief A Layer in a feed forward neural network.
*/
class Layer{
    public:
        Layer();
        Layer(json layer_config, float random);
        /**
         * @brief A function to determine the outputs in a layer.
         * @param inputs A vector of float inputs.
        */
        void determine_output(std::vector<float> inputs);
        /**
         * @brief A function to determine the outputs in a layer.
         * @param inputs A vector of ff Node inputs.
        */
        void determine_output(std::vector<Node> input);
        
        /**
         * @brief Get the outputs from each node in a layer.
         * @return A vector of floats representing the node output.
        */
        std::vector<float> get_outputs();
        /**
         * @brief A vector of nodes.
        */
        std::vector<Node> nodes;

        /** @brief Calculates the current weights and biases that the neural net is using.
         * @return The json object
        */
        json get_json();
    private:
        /**
         * @brief The number of nodes in the layer.
        */
        int num_nodes;
};
}}