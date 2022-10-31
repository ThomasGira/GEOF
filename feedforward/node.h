#pragma once

#include <vector>

namespace geoff{
namespace ff{
/**
 * @brief A node in a feed forward neural network.
*/
class Node{
    public:
        Node();
        Node(std::vector<float>weights, float bias, float random);
        /**
         * @brief A function to determine the output from a node based on a Vector of equal length to the number of weights.
         * @param inputs A vector of float inputs.
        */
        void determine_output(std::vector<float> inputs);
        /**
         * @brief A function to determine the output from a node based on a Vector of equal length to the number of weights.
         * @param inputs A vector of ff Node inputs.
        */
        void determine_output(std::vector<Node> input);

        /**
         * @brief The resultant output from inputs.
        */
        float output;
    private:
        /**
         * @brief A vector of weights to apply to an input number.
        */
        std::vector<float> weights;

        /**
         * @brief The number of input variables.
        */
        int length;

        /**
         * @brief The bias to apply when calculating the output.
        */
        float bias;

        /**
         * @brief Calculate the sigmoid valaue of a function.
         * @param input The number to apply a sigmoid function to.
         * @return The applied value.
        */
        float sigmoid(float);
};
}}