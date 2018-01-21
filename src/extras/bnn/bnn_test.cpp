
#include "bnn.hpp"

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


struct handwrittenBNN
{
    size_t num_input_;
    size_t num_output_;
    vector<double> weights_;
    const double BIAS_VAL = 1.0;

    handwrittenBNN(size_t NI, size_t NO, const vector<double> & W) :
        num_input_(NI),
        num_output_(NO),
        weights_(W) {

        assert(W.size() == (NI + 1)*NO);
    }

    vector<double> activate(vector<double> input) {
        input.push_back(BIAS_VAL);

        vector<double> output_in;
        for (size_t o = 0; o < num_output_; ++o) {
            double o_val = 0;
            for (size_t i = 0; i < input.size(); ++i) {
                o_val += input.at(i) * weights_.at(i + (o * input.size()));
            }
            output_in.push_back(o_val);
        }

        vector<double> output_out;
        for (const auto & val : output_in) output_out.push_back(sigmoid(val));
        return output_out;
    }


    double sigmoid(double x) {
        return 1.0 / (1.0 + std::exp(-1 * x));
    }


};


int main()
{

    static size_t rseed = 0;
    static std::mt19937 reng(rseed);
    static std::uniform_int_distribution<long> uni_int(3, 100);

    std::chrono::duration<double> eigen_time{0};
    std::chrono::duration<double> handwritten_time{0};

    for (int trial = 0; trial < 1000; ++trial)
    {
        auto ni = uni_int(reng);
        auto no = uni_int(reng);

        Eigen::MatrixXd w_a = Eigen::MatrixXd::Random((ni + 1)*no, 1);
        Eigen::MatrixXd input_a = Eigen::MatrixXd::Random(1, ni);
        vector<double> w_b(w_a.data(), w_a.data() + w_a.rows() * w_a.cols());
        vector<double> input_b(input_a.data(), input_a.data() + input_a.rows() * input_a.cols());

        BNN a(ni, no, w_a);
        handwrittenBNN b(ni, no, w_b);

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
