#ifndef CASADI_EIGEN_UTILS_H
#define CASADI_EIGEN_UTILS_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <vector>

/**
 * @brief Convert Eigen::VectorXd to casadi::DM
 */
inline casadi::DM eigen_to_casadi(const Eigen::VectorXd& vec) {
    std::vector<double> data(vec.data(), vec.data() + vec.size());
    return casadi::DM(data);
}

/**
 * @brief Convert Eigen::MatrixXd to casadi::DM
 */
inline casadi::DM eigen_to_casadi(const Eigen::MatrixXd& mat) {
    std::vector<double> data(mat.data(), mat.data() + mat.size());
    return casadi::DM::reshape(casadi::DM(data), mat.rows(), mat.cols());
}

/**
 * @brief Convert Eigen::Matrix4d to casadi::DM
 */
inline casadi::DM eigen_to_casadi(const Eigen::Matrix4d& mat) {
    std::vector<double> data;
    data.reserve(16);
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            data.push_back(mat(row, col));
        }
    }
    return casadi::DM::reshape(casadi::DM(data), 4, 4);
}

/**
 * @brief Convert casadi::DM to Eigen::VectorXd
 */
inline Eigen::VectorXd casadi_to_eigen_vector(const casadi::DM& dm) {
    std::vector<double> data = static_cast<std::vector<double>>(dm);
    return Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
}

/**
 * @brief Convert casadi::DM to Eigen::MatrixXd
 */
inline Eigen::MatrixXd casadi_to_eigen_matrix(const casadi::DM& dm) {
    std::vector<double> data = static_cast<std::vector<double>>(dm);
    int rows = dm.rows();
    int cols = dm.columns();
    
    Eigen::MatrixXd mat(rows, cols);
    for (int col = 0; col < cols; ++col) {
        for (int row = 0; row < rows; ++row) {
            mat(row, col) = data[col * rows + row];
        }
    }
    return mat;
}

#endif // CASADI_EIGEN_UTILS_H