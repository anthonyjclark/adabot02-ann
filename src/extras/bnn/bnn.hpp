
#ifndef NN_HPP
#define NN_HPP

#include <Eigen/Dense>
#include <cassert>
#include <cmath>



// Weights array to matrix (3, 3)
// Index : Input -> Hidden
// 0     : 0     -> 0
// 1     : 1     -> 0
// 2     : 2     -> 0
// 3     : B     -> 0
// 4     : 0     -> 1
// 5     : 1     -> 1
// 6     : 2     -> 1
// 7     : B     -> 1
// 8     : 0     -> 2
// 9     : 1     -> 2
// 0     : 2     -> 2
// 1     : B     -> 2


// #include <iostream>


// http://eigen.tuxfamily.org/dox/group__QuickRefPage.html#title6
// https://stats.stackexchange.com/questions/115258/
// comprehensive-list-of-activation-functions-in-neural-networks-with-pros-cons

inline Eigen::MatrixXd sigmoid_act(const Eigen::MatrixXd & x) {
    return 1.0 / (1.0 + (-1 * x).array().exp());
}

inline Eigen::MatrixXd tanh_act(const Eigen::MatrixXd & x) {
    return x.array().tanh();
}

inline Eigen::MatrixXd relu_act(const Eigen::MatrixXd & x) {
    return x.array().max(0);
}


class BNN
{

public:

    long num_input_;
    long num_output_;

    Eigen::MatrixXd weights_;

    const double BIAS_VAL = 1.0;

    std::function<Eigen::MatrixXd(Eigen::MatrixXd x)> activation_function;
    // 0 --> sigmoid_act
    // 1 --> tanh_act
    // 2 --> relu_act

public:

    BNN(long NI, long NO, const Eigen::MatrixXd & W, size_t act=0) :
        num_input_(NI),
        num_output_(NO),
        weights_(NI + 1, NO) {

        assert(W.size() == (NI + 1) * NO);

        for (int i = 0; i < W.size(); ++i) weights_(i) = W(i);

        // Set the activation function
        if (act == 0) activation_function = sigmoid_act;
        else if (act == 1) activation_function = tanh_act;
        else if (act == 2) activation_function = relu_act;
        else activation_function = sigmoid_act;
    }


    Eigen::MatrixXd activate(Eigen::MatrixXd input) {
        input.conservativeResize(Eigen::NoChange, input.cols() + 1);
        input(input.size() - 1) = BIAS_VAL;
        // std::cout << input.rows() << ", " << input.cols() << std::endl;
        // std::cout << weights_.rows() << ", " << weights_.cols() << std::endl;
        return activation_function(input * weights_);
    }
};

#endif
