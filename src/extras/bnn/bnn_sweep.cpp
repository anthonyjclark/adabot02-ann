
#include "bnn.hpp"

#include <iostream>
using std::cout;
using std::endl;
using std::cerr;

int main(int argc, char const *argv[])
{
    if (argc < 2) {
        cerr << "A string of weights is expected." << endl;
        return 1;
    }

    constexpr size_t NI = 3;
    constexpr size_t NO = 3;
    constexpr size_t N = (NI + 1) * NO;

    // Read the activation type and weights from the program argument
    std::istringstream iss{argv[1]};

    size_t act;
    iss >> act;

    double val; long w_idx = 0;
    Eigen::MatrixXd weights(N, 1);
    while (iss >> val) {
        weights(w_idx++) = val;
    }

    // while (w_idx < N) {
    //     weights(w_idx++) = 0;
    // }
    // weights(12) = 1;

    if (w_idx != N) {
        cerr << "Incorrect number of weights." << endl;
        return 1;
    }

    // Sweep the input space
    BNN bnn(NI, NO, weights, act);
    constexpr double STEP_SIZE = 0.1;
    Eigen::MatrixXd nn_input(1, NI);

    cout << "Angle,Angular Error,Linear Error,Left Speed,Right Speed,Extension" << endl;
    for (double angle = 0; angle <= 1; angle += STEP_SIZE) {
        for (double angular = 0; angular <= 1; angular += STEP_SIZE) {
            for (double linear = 0; linear <= 1; linear += STEP_SIZE) {

                nn_input(0) = angle;
                nn_input(1) = angular;
                nn_input(2) = linear;

                Eigen::MatrixXd nn_output = bnn.activate(nn_input);
                Eigen::MatrixXd nn_output_clamped = nn_output.array().min(1).max(0);

                cout << angle << "," << angular << "," << linear
                     << "," << nn_output_clamped(0)
                     << "," << nn_output_clamped(1)
                     << "," << nn_output_clamped(2) << endl;
            }
        }
    }

    return 0;
}
