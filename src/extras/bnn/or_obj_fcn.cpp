

#include "bnn.hpp"

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
    long NI, NO, ACT;
    iss >> NI >> NO >> ACT;

    // Read weights
    double val;
    long w_idx = 0;
    Eigen::MatrixXd weights((NI + 1)*NO, 1);
    while (iss >> val) {
        weights(w_idx++) = val;
    }

    Eigen::MatrixXd input1(1, 2), input2(1, 2), input3(1, 2), input4(1, 2);
    input1 << 0, 0;
    input2 << 0, 1;
    input3 << 1, 0;
    input4 << 1, 1;

    BNN bnn(NI, NO, weights, ACT);

    double fitness = abs(bnn.activate(input1)(0))
                   + abs(1 - bnn.activate(input2)(0))
                   + abs(1 - bnn.activate(input3)(0))
                   + abs(1 - bnn.activate(input4)(0));

    std::cout << fitness;

    return 0;
}
