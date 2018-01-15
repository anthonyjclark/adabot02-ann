
#include "nn.hpp"

#include <iostream>
using std::cout;
using std::endl;

#include <vector>
using std::vector;

#include <string>
using std::string;

#include <sstream>
using std::stringstream;

#include <random>

// Interesting results without -O3

string vec_to_string(vector<double> v) {
    stringstream ss;
    for (const auto & val : v) ss << val << "  ";
    return ss.str();
}


bool all_close(Eigen::MatrixXd a, vector<double> b) {
    if (a.size() != static_cast<long>(b.size())) return false;
    for (size_t i = 0; i < b.size(); ++i) {
        if (std::abs(a(i) - b.at(i)) > 0.000001) {
            return false;
        }
    }
    return true;
}


struct handwrittenNN
{
    size_t num_input_;
    size_t num_hidden_;
    size_t num_output_;
    vector<double> weights_input_hidden_;
    vector<double> weights_hidden_output_;
    const double BIAS_VAL = 1.0;

    handwrittenNN(size_t NI, size_t NH, size_t NO, vector<double> W) :
        num_input_(NI),
        num_hidden_(NH),
        num_output_(NO) {

        assert(W.size() == ((NI + 1)*NH + (NH + 1)*NO));

        size_t w_idx = 0;
        // Fill input to hidden
        for (; w_idx < (NI + 1)*NH; ++w_idx) {
            weights_input_hidden_.push_back(W.at(w_idx));
        }

        // Fill hidden to output
        for (; w_idx < W.size(); ++w_idx) {
            weights_hidden_output_.push_back(W.at(w_idx));
        }
    }

    vector<double> activate(vector<double> input) {
        input.push_back(BIAS_VAL);
        // cout << "input      : " << vec_to_string(input) << endl;

        vector<double> hidden_in;
        for (size_t h = 0; h < num_hidden_; ++h) {
            double h_val = 0;
            for (size_t i = 0; i < input.size(); ++i) {
                h_val += input.at(i) * weights_input_hidden_.at(i + (h * input.size()));
            }
            hidden_in.push_back(h_val);
        }
        // cout << "hidden_in  : " << vec_to_string(hidden_in) << endl;

        vector<double> hidden_out;
        for (const auto & val : hidden_in) hidden_out.push_back(sigmoid(val));
        hidden_out.push_back(BIAS_VAL);
        // cout << "hidden_out : " << vec_to_string(hidden_out) << endl;

        vector<double> output_in;
        for (size_t o = 0; o < num_output_; ++o) {
            double o_val = 0;
            for (size_t h = 0; h < hidden_out.size(); ++h) {
                o_val += hidden_out.at(h) * weights_hidden_output_.at(h + (o * hidden_out.size()));
            }
            output_in.push_back(o_val);
        }
        // cout << "output_in  : " << vec_to_string(output_in) << endl;

        vector<double> output_out;
        for (const auto & val : output_in) output_out.push_back(sigmoid(val));
        // cout << "output_out : " << vec_to_string(output_out) << endl;

        return output_out;
    }


    double sigmoid(double x) {
        return 1.0 / (1.0 + std::exp(-1 * x));
    }


};


int main()
{
    // long ni = 2;
    // long nh = 2;
    // long no = 3;

    // Eigen::MatrixXd w = Eigen::MatrixXd::Random((ni + 1)*nh + (nh + 1)*no, 1);
    // Eigen::MatrixXd w = Eigen::MatrixXd::Ones((ni + 1)*nh + (nh + 1)*no, 1);

    // NN a(ni, nh, no, w);

    // cout << "num_input_  : " << a.num_input_ << "\n"
    //      << "num_hidden_ : " << a.num_hidden_ << "\n"
    //      << "num_output_ : " << a.num_output_ << "\n"
    //      << "\nweights_input_hidden_  :\n" << a.weights_input_hidden_ << "\n"
    //      << "\nweights_hidden_output_ :\n" << a.weights_hidden_output_ << endl;

    // Eigen::MatrixXd input = Eigen::MatrixXd::Random(1, ni);
    // cout << "\nOutput :\n" << a.activate(input) << "\n" << endl;
    // a.activate_verbose(input);

    // cout << "----------------------------------------------------------------" << endl;

    // --------------------------------------------------------------------
    // --------------------------------------------------------------------
    // --------------------------------------------------------------------

    // vector<double> w2(w.data(), w.data() + w.rows() * w.cols());
    // handwrittenNN b(ni, nh, no, w2);

    // cout << "\nnum_input_  : " << b.num_input_ << "\n"
    //      << "num_hidden_ : " << b.num_hidden_ << "\n"
    //      << "num_output_ : " << b.num_output_ << "\n"
    //      << "\nweights_input_hidden_  :\n" << vec_to_string(b.weights_input_hidden_) << "\n"
    //      << "\nweights_hidden_output_ :\n" << vec_to_string(b.weights_hidden_output_) << endl;

    // vector<double> input2(input.data(), input.data() + input.rows() * input.cols());
    // b.activate(input2);
    // cout << "\nOutput :\n" << vec_to_string(b.activate(input2)) << endl;


    std::cout << "x sigmoid tanh relu" << std::endl;
    for (double x = -5.0; x < 5.0; x += 0.1) {
        Eigen::MatrixXd x_mat(1, 1);
        x_mat << x;
        auto fx_sigmoid = sigmoid_act(x_mat);
        auto fx_tanh = tanh_act(x_mat);
        auto fx_relu = relu_act(x_mat);
        std::cout << x
                  << " " << fx_sigmoid
                  << " " << fx_tanh
                  << " " << fx_relu << std::endl;
    }

    return 0;


    static size_t rseed = 0;
    static std::mt19937 reng(rseed);
    static std::uniform_int_distribution<long> uni_int(3, 100);

    std::chrono::duration<double> eigen_time{0};// = 0;
    std::chrono::duration<double> handwritten_time{0};// = 0;

    for (int trial = 0; trial < 1000; ++trial)
    {
        auto ni = uni_int(reng);
        auto nh = uni_int(reng);
        auto no = uni_int(reng);

        Eigen::MatrixXd w_a = Eigen::MatrixXd::Random((ni + 1)*nh + (nh + 1)*no, 1);
        Eigen::MatrixXd input_a = Eigen::MatrixXd::Random(1, ni);
        vector<double> w_b(w_a.data(), w_a.data() + w_a.rows() * w_a.cols());
        vector<double> input_b(input_a.data(), input_a.data() + input_a.rows() * input_a.cols());

        NN a(ni, nh, no, w_a);
        handwrittenNN b(ni, nh, no, w_b);

        auto start = std::chrono::system_clock::now();
        auto out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        out_a = a.activate(input_a);
        auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        eigen_time += elapsed_seconds;

        start = std::chrono::system_clock::now();
        auto out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        out_b = b.activate(input_b);
        end = std::chrono::system_clock::now();

        elapsed_seconds = end - start;
        handwritten_time += elapsed_seconds;

        if (!all_close(out_a, out_b)) {
            cout << "Not the same" << endl;
            break;
        }

        cout << trial << endl;
    }

    cout << "Eigen time       : " << eigen_time.count() << endl;
    cout << "Handwritten time : " << handwritten_time.count() << endl;

    return 0;
}
