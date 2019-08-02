#include "ssa.h"

namespace kamaz {
namespace hagen {

    SingularSpectrumAnalysis::SingularSpectrumAnalysis(Eigen::VectorXf input_signal, int win_size){
        feature_vector = input_signal;
        M = win_size;
        N = feature_vector.size(); 
        number_of_lags = N - M + 1;
    }

    void SingularSpectrumAnalysis::normalize(){
        auto mean  =  feature_vector.mean();
        auto centering = feature_vector.array() - mean;
        auto standad_dev = std::sqrt(centering.square().sum()/(N)); 
        if(standad_dev != 0){
            feature_vector = centering.array() / standad_dev ;
        }
        // std::cout<< "feature_vector: "<< feature_vector.transpose() << std::endl; 
    }

    void SingularSpectrumAnalysis::calculate_trajectory_matrix(){
        trajectory_matrix = Eigen::MatrixXf::Zero(number_of_lags, M);
        for(int i=0; i<number_of_lags; i++){
            trajectory_matrix.row(i) = feature_vector.segment(i, M);
        }
        // std::cout<< "trajectory_matrix: "<< trajectory_matrix << std::endl;
    }

    void SingularSpectrumAnalysis::calculate_eigne_vectors_and_values(){
        
        Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;
        eigensolver.compute(covariance_matrix);
        Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
        Eigen::MatrixXf eigen_vectors = eigensolver.eigenvectors().real();
        // std::cout<< "eigen_vectors" << eigen_vectors.real() << std::endl;
        // std::cout<< "eigen_values" << eigen_values.real() << std::endl;

        for(int i=0; i<eigen_values.size(); i++){
            std::tuple<float, Eigen::VectorXf> vec_and_val(eigen_values[i], eigen_vectors.row(i));
            eigen_vectors_and_values.push_back(vec_and_val);
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(), 
            [&](const std::tuple<float, Eigen::VectorXf>& a, const std::tuple<float, Eigen::VectorXf>& b) -> bool{ 
                return std::get<0>(a) > std::get<0>(b); 
        });
        // std::cout<< "eigen_vectors_and_values" << std::endl;
        // for(auto const vec : eigen_vectors_and_values){
        //     std::cout<< std::get<0>(vec)<< std::endl;
        //     std::cout<< "===========" << std::endl;
        //     std::cout<< std::get<1>(vec)<< std::endl;
        // }
    }

    void SingularSpectrumAnalysis::calculate_principle_components(){
        int number_of_values = eigen_vectors_and_values.size();
        if(number_of_values<1){
            std::cout<< "No eiegen values has found" << std::endl;
            return;
        }
        eigen_vectors = Eigen::MatrixXf::Zero(number_of_values, std::get<1>(eigen_vectors_and_values[0]).size());
        for(int i=0; i<number_of_values; i++){
            eigen_vectors.row(i) = std::get<1>(eigen_vectors_and_values[i]);
        }
        // eigen_vectors = eigen_vectors.transpose();
        // std::cout<< "eigen_vectors: "<< eigen_vectors<< std::endl;
        principal_components = trajectory_matrix*eigen_vectors;
        // std::cout<< "principal_components: "<< principal_components << std::endl;
    }

    void SingularSpectrumAnalysis::reconstruct_matrix(){
        if(eigen_vectors_and_values.size()<1){
            std::cout<< "No eiegen values has found" << std::endl;
            return;
        }
        reconstructed_matrix = Eigen::MatrixXf::Zero(N, M);
        for(auto m(0); m<M; m++){
            Eigen::MatrixXf buf = principal_components.col(m)*eigen_vectors.col(m).transpose();
            Eigen::MatrixXf reversed_buf = Eigen::MatrixXf::Zero(buf.rows(), buf.cols());
            for(auto k(0); k<buf.rows(); k++){
                reversed_buf.row(k) = buf.row(buf.rows()-k-1);
            }
            for(auto n(0); n<N; n++){
                reconstructed_matrix(n,m) = reversed_buf.diagonal(-(N-M)+n).real().mean();
            }
        }
        // std::cout<< "reconstructed_matrix: "<< reconstructed_matrix << std::endl;
    }

    Eigen::VectorXf SingularSpectrumAnalysis::get_reconstructed_signal(int number_comps){
        // Eigen::MatrixXf gg = reconstructed_matrix.transpose();
        Eigen::VectorXf reconstructed_final_signal
         = reconstructed_matrix.block(0, 0, reconstructed_matrix.rows(), number_comps).rowwise().sum();
        return reconstructed_final_signal;
    }

    void SingularSpectrumAnalysis::calculate_covariance_matrix_toeplitz(){
        Eigen::VectorXf correlation = cross_corelation(feature_vector, feature_vector, M-1);
        Eigen::VectorXf coff = correlation.segment(M-1, M);
        covariance_matrix = get_toeplitz_matrix(coff, coff);
        // std::cout<< "covariance_matrix: "<< covariance_matrix << std::endl;
    }

    Eigen::MatrixXf SingularSpectrumAnalysis::get_toeplitz_matrix(Eigen::VectorXf c, Eigen::VectorXf r){
        int size_c = c.size();
        int size_r = r.size();
        Eigen::MatrixXf toeplitz_matrix = Eigen::MatrixXf::Zero(size_c, size_r);
        for(auto i(0); i<size_c; i++){
            int index_c = i;
            for(auto j(0); j<=i; j++){
                toeplitz_matrix(i, j) = c[index_c];
                index_c--;
            }
            int index_r = 1;
            for(auto k(i+1); k<size_r; k++){
                toeplitz_matrix(i, k) = r[index_r];
                index_r++;
            }
        }
        return toeplitz_matrix;
    }

    Eigen::VectorXf SingularSpectrumAnalysis::cross_corelation(Eigen::VectorXf x, Eigen::VectorXf y, int max_lags){
        auto Nx = x.size();
        auto Ny = y.size();
        if(Nx != Ny){
            std::cout<<" Both vectors must have same dimentions" << std::endl;
        }
        int n = Nx + Ny -1;
        Eigen::VectorXf c(n);
        for(int j=0; j<Ny; j++){
            float sum_all = 0;
            for(int k=0; k<=j; k++){
                sum_all += y[k]*x[Nx-j-1+k];
            }
            c[j] = sum_all;
        }
        int j=0;
        for(auto i(Ny); i< n; i++){
            c[i] = c[Ny-2-j];
            j++;
        }
        float factor = std::sqrt(x.dot(x) * y.dot(y));
        if(factor != 0){
            c = c.array()/factor;
        }
        if(max_lags >= Nx || max_lags < 1){
            std::cout<<" Max_lags should be positive and less than given feature vector" << std::endl;
        }
        auto points =  2*max_lags+1;
        // std::cout<< c.transpose() << std::endl;
        Eigen::VectorXf res = c.segment(Nx-1-max_lags, points);
        return res;
    }

    Eigen::VectorXf SingularSpectrumAnalysis::conv(Eigen::VectorXf f, Eigen::VectorXf g) {
        int const nf = f.size();
        int const ng = g.size();
        int const n  = nf + ng - 1;
        Eigen::VectorXf out(n);
        for(auto i(0); i < n; ++i) {
            int const jmn = (i >= ng - 1)? i - (ng - 1) : 0;
            int const jmx = (i <  nf - 1)? i : nf - 1;
            for(auto j(jmn); j <= jmx; ++j) {
                out[i] += (f[j] * g[i - j]);
            }
        }
        return out; 
    }

    Eigen::VectorXf SingularSpectrumAnalysis::execute(int number_of_components, bool is_normalized){
        if(is_normalized){
            normalize();
        }
        calculate_trajectory_matrix();
        calculate_covariance_matrix_toeplitz();
        calculate_eigne_vectors_and_values();
        if(eigen_vectors_and_values.size()<1){
            return feature_vector;
        }
        calculate_principle_components();
        reconstruct_matrix();
        return get_reconstructed_signal(number_of_components);
    }

    void SingularSpectrumAnalysis::save_data(int i){
        save_vec(feature_vector, "feature_vector_" + std::to_string(i));
        save_mat(covariance_matrix, "covariance_matrix_" + std::to_string(i));
        save_mat(trajectory_matrix, "trajectory_matrix_" + std::to_string(i));
        save_mat(principal_components, "principal_components_" + std::to_string(i));
        save_mat(reconstructed_matrix, "reconstructed_matrix_" + std::to_string(i));
        save_mat(eigen_vectors, "eigen_vectors_" + std::to_string(i));
    }

    void SingularSpectrumAnalysis::save_vec(Eigen::VectorXf vec, std::string name){
        std::vector<float> feature_vector_np;
        int size_of = vec.size(); 
        for(auto i(0); i<size_of; i++){
            feature_vector_np.push_back(vec[i]);
        }
        std::string location = "/dataset/result/" + name + ".npy";
        cnpy::npy_save(location ,&feature_vector_np[0],{(unsigned int)1, (unsigned int)size_of, (unsigned int)1},"w");
    }

    void SingularSpectrumAnalysis::save_mat(Eigen::MatrixXf matrix, std::string name){
        std::vector<float> matrix_np;
        int rows = matrix.rows(); 
        int cols = matrix.cols(); 
        for(auto i(0); i<rows; i++){
            for(auto j(0); j<cols; j++){
                matrix_np.push_back(matrix(i,j));
            }
        }
        std::string location = "/dataset/result/" + name + ".npy";
        cnpy::npy_save(location ,&matrix_np[0],{(unsigned int)1, (unsigned int)rows, (unsigned int)cols},"w");
    }
    
}
}