#include "include/weighted_moving_filter.h"
#include <iostream>
#include <stdexcept>

WeightedMovingFilter::WeightedMovingFilter(const Eigen::VectorXd& weights, int data_size)
    : window_size_(weights.size()),
      weights_(weights),
      data_size_(data_size),
      filtered_data_(Eigen::VectorXd::Zero(data_size)) {
    
    // Verify that weights sum to 1.0
    double sum = weights_.sum();
    if (std::abs(sum - 1.0) > 1e-6) {
        throw std::invalid_argument("[WeightedMovingFilter] the sum of weights list must be 1.0!");
    }
}

Eigen::VectorXd WeightedMovingFilter::apply_filter() {
    if (data_queue_.size() < static_cast<size_t>(window_size_)) {
        return data_queue_.back();
    }
    
    Eigen::VectorXd temp_filtered_data = Eigen::VectorXd::Zero(data_size_);
    
    // Apply convolution for each dimension
    for (int i = 0; i < data_size_; ++i) {
        double sum = 0.0;
        for (int j = 0; j < window_size_; ++j) {
            int data_idx = data_queue_.size() - window_size_ + j;
            sum += data_queue_[data_idx](i) * weights_(j);
        }
        temp_filtered_data(i) = sum;
    }
    
    return temp_filtered_data;
}

void WeightedMovingFilter::add_data(const Eigen::VectorXd& new_data) {
    if (new_data.size() != data_size_) {
        throw std::invalid_argument("New data size does not match expected data size");
    }
    
    // Skip duplicate data
    if (!data_queue_.empty() && new_data.isApprox(data_queue_.back())) {
        return;
    }
    
    // Remove oldest data if queue is full
    if (data_queue_.size() >= static_cast<size_t>(window_size_)) {
        data_queue_.erase(data_queue_.begin());
    }
    
    data_queue_.push_back(new_data);
    filtered_data_ = apply_filter();
}
