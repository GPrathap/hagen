#ifndef GROUND_REMOVAL_SSA_H_
#define GROUND_REMOVAL_SSA_H_

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>
#include <cnpy.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Eigenvalues> 
#include <complex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

namespace kamaz {
namespace hagen {
        class SingularSpectrumAnalysis {
            public:
                SingularSpectrumAnalysis(Eigen::VectorXf input_signal, int win_size);
                ~SingularSpectrumAnalysis() = default; 
                Eigen::VectorXf execute(int number_of_components, bool is_normalized);
                void save_data(int i);
                void save_vec(Eigen::VectorXf vec, std::string name);

            private:
                void normalize();
                void calculate_trajectory_matrix();
                Eigen::VectorXf  cross_corelation(Eigen::VectorXf x, Eigen::VectorXf y, int max_lags);
                Eigen::VectorXf conv(Eigen::VectorXf f, Eigen::VectorXf g);
                void calculate_covariance_matrix_toeplitz();
                Eigen::MatrixXf get_toeplitz_matrix(Eigen::VectorXf c, Eigen::VectorXf r);
                void calculate_eigne_vectors_and_values();
                void calculate_principle_components();
                void reconstruct_matrix();
                void save_mat(Eigen::MatrixXf matrix, std::string name);

                Eigen::VectorXf get_reconstructed_signal(int number_comps);
                std::vector<std::tuple<float, Eigen::VectorXf>> eigen_vectors_and_values; 
                Eigen::VectorXf feature_vector;
                Eigen::MatrixXf covariance_matrix;
                Eigen::MatrixXf trajectory_matrix;
                Eigen::MatrixXf principal_components;
                Eigen::MatrixXf reconstructed_matrix;
                Eigen::MatrixXf eigen_vectors;
                int M;
                int N;
                int number_of_lags;
            };
    }
}
#endif