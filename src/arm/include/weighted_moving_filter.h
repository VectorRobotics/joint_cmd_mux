#ifndef WEIGHTED_MOVING_FILTER_H
#define WEIGHTED_MOVING_FILTER_H

#include <Eigen/Dense>
#include <vector>
#include <cassert>
#include <cmath>

class WeightedMovingFilter {
public:
    /**
     * @brief Constructor for WeightedMovingFilter
     * @param weights Vector of weights for the moving average (must sum to 1.0)
     * @param data_size Dimension of the data to be filtered
     */
    WeightedMovingFilter(const Eigen::VectorXd& weights, int data_size = 14);
    
    /**
     * @brief Add new data to the filter and compute filtered result
     * @param new_data Vector of data to add (must have size equal to data_size)
     */
    void add_data(const Eigen::VectorXd& new_data);
    
    /**
     * @brief Get the current filtered data
     * @return The filtered data vector
     */
    Eigen::VectorXd filtered_data() const { return filtered_data_; }

private:
    /**
     * @brief Apply the weighted moving average filter
     * @return The filtered data vector
     */
    Eigen::VectorXd apply_filter();

    int window_size_;
    Eigen::VectorXd weights_;
    int data_size_;
    Eigen::VectorXd filtered_data_;
    std::vector<Eigen::VectorXd> data_queue_;
};

#endif // WEIGHTED_MOVING_FILTER_H
