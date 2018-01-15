
#ifndef NN_HPP
#define NN_HPP

#include <Eigen/Dense>
#include <cassert>
#include <cmath>

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


class NN
{

public:

    long num_input_;
    long num_hidden_;
    long num_output_;

    Eigen::MatrixXd weights_input_hidden_;

    Eigen::MatrixXd weights_hidden_output_;

    const double BIAS_VAL = 1.0;

    std::function<Eigen::MatrixXd(Eigen::MatrixXd x)> activation_function;
    // 0 --> sigmoid_act
    // 1 --> tanh_act
    // 2 --> relu_act

public:

    NN(long NI, long NH, long NO, Eigen::MatrixXd W, size_t act=0) :
        num_input_(NI),
        num_hidden_(NH),
        num_output_(NO),
        weights_input_hidden_(NI + 1, NH),
        weights_hidden_output_(NH + 1, NO) {

        assert(W.size() == ((NI + 1)*NH + (NH + 1)*NO));

        long w_idx = 0;
        // Fill input to hidden
        for (long ih_idx = 0; ih_idx < weights_input_hidden_.size(); ++w_idx, ++ih_idx) {
            weights_input_hidden_(ih_idx) = W(w_idx);
        }

        // Fill hidden to output
        for (long ho_idx = 0; ho_idx < weights_hidden_output_.size(); ++w_idx, ++ho_idx) {
            weights_hidden_output_(ho_idx) = W(w_idx);
        }

        // Set the activation function
        if (act == 0) activation_function = sigmoid_act;
        else if (act == 1) activation_function = tanh_act;
        else if (act == 2) activation_function = relu_act;
        else activation_function = sigmoid_act;
    }


    Eigen::MatrixXd activate(Eigen::MatrixXd input) {

        input.conservativeResize(Eigen::NoChange, input.cols() + 1);
        input(input.size() - 1) = BIAS_VAL;
        auto hidden = activation_function(input * weights_input_hidden_);

        hidden.conservativeResize(Eigen::NoChange, hidden.cols() + 1);
        hidden(hidden.size() - 1) = BIAS_VAL;
        return activation_function(hidden * weights_hidden_output_);;
    }
};

#endif
