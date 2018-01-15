

#include "nn.hpp"

#include <sstream>
#include <iostream>
#include <vector>

int main(int argc, char const *argv[])
{
    // Require at least 1 program argument
    if (argc < 2) {
        return 1;
    }

    // Treat command line argument as string
    std::istringstream iss(argv[1]);

    // Read NN structure
    long NI, NH, NO, ACT;
    iss >> NI >> NH >> NO >> ACT;

    // Read weights
    double val;
    long w_idx = 0;
    Eigen::MatrixXd weights((NI + 1)*NH + (NH + 1)*NO, 1);
    while (iss >> val) {
        weights(w_idx++) = val;
    }

    // std::cout << "Num input nodes  : " << NI << std::endl;
    // std::cout << "Num hidden nodes : " << NH << std::endl;
    // std::cout << "Num output nodes : " << NO << std::endl;
    // std::cout << "Weights :\n" << weights << std::endl;

    Eigen::MatrixXd input1(1, 2), input2(1, 2), input3(1, 2), input4(1, 2);
    input1 << 0, 0;
    input2 << 0, 1;
    input3 << 1, 0;
    input4 << 1, 1;

    NN ann(NI, NH, NO, weights, ACT);

    // std::cout << ann.activate(input1) << std::endl;
    // std::cout << ann.activate(input2) << std::endl;
    // std::cout << ann.activate(input3) << std::endl;
    // std::cout << ann.activate(input4) << std::endl;

    double fitness = abs(ann.activate(input1)(0))
                   + abs(1 - ann.activate(input2)(0))
                   + abs(1 - ann.activate(input3)(0))
                   + abs(ann.activate(input4)(0));

    std::cout << fitness;

    return 0;
}
