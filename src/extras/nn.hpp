
#ifndef NN_HPP
#define NN_HPP

#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include <cmath>

class NN
{

public:

    long num_input_;
    long num_hidden_;
    long num_output_;

    Eigen::MatrixXd weights_input_hidden_;

    Eigen::MatrixXd weights_hidden_output_;

    const double BIAS_VAL = 1.0;

public:

    NN(long NI, long NH, long NO, Eigen::MatrixXd W) :
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
    }


    Eigen::MatrixXd activate(Eigen::MatrixXd input) {

        input.conservativeResize(Eigen::NoChange, input.cols() + 1);
        input(input.size() - 1) = BIAS_VAL;
        auto hidden = sigmoid(input * weights_input_hidden_);

        hidden.conservativeResize(Eigen::NoChange, hidden.cols() + 1);
        hidden(hidden.size() - 1) = BIAS_VAL;
        return sigmoid(hidden * weights_hidden_output_);;
    }


    Eigen::MatrixXd sigmoid(Eigen::MatrixXd x) {
        return 1.0 / (1.0 + (-1 * x).array().exp());
    }
};

#endif
