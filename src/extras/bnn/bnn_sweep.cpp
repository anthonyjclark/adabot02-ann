
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

    if (w_idx != N) {
        cerr << "Incorrect number of weights." << endl;
        return 1;
    }

    // Sweep the input space
    BNN nn(NI, NO, weights, act);
    constexpr double STEP_SIZE = 0.1;
    Eigen::MatrixXd nn_input(1, NI);

    cout << "angle,angular error,linear error,left speed,right speed,extension" << endl;
    for (double angle = 0; angle <= 1; angle += STEP_SIZE) {
        for (double angular = 0; angular <= 1; angular += STEP_SIZE) {
            for (double linear = 0; linear <= 1; linear += STEP_SIZE) {

                nn_input(0) = angle * 3.1415926;
                nn_input(1) = angular;
                nn_input(2) = linear;

                auto nn_output = nn.activate(nn_input);

                cout << angle << "," << angular << "," << linear
                     << "," << nn_output(0)
                     << "," << nn_output(1)
                     << "," << nn_output(2) << endl;
            }
        }
    }

    return 0;
}
