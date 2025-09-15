#include "state_estimation/MovingHorizonEstimation.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace state_estimation {

MovingHorizonEstimation::MovingHorizonEstimation(size_t state_dim, size_t measurement_dim,
                                               size_t control_dim, size_t horizon_length)
    : state_dim_(state_dim),
      measurement_dim_(measurement_dim),
      control_dim_(control_dim),
      horizon_length_(horizon_length),
      is_initialized_(false),
      x_current_(state_dim, 0.0),
      state_trajectory_(horizon_length, std::vector<double>(state_dim, 0.0)),
      arrival_cost_state_(state_dim, 0.0),
      arrival_cost_cov_(state_dim, std::vector<double>(state_dim, 0.0)),
      Q_(state_dim, std::vector<double>(state_dim, 0.0)),
      R_(measurement_dim, std::vector<double>(measurement_dim, 0.0)),
      Q_inv_(state_dim, std::vector<double>(state_dim, 0.0)),
      R_inv_(measurement_dim, std::vector<double>(measurement_dim, 0.0)) {
    
    // TODO: Implement parameter validation
}

void MovingHorizonEstimation::initialize(const std::vector<double>& initial_state,
                                        const std::vector<std::vector<double>>& initial_covariance) {
    // TODO: Implement initialization
}

void MovingHorizonEstimation::setModels(const ProcessModel& f, const MeasurementModel& h,
                                       const OptimizationSolver& solver) {
    // TODO: Implement model setting
}

void MovingHorizonEstimation::setNoiseCovariances(
    const std::vector<std::vector<double>>& Q,
    const std::vector<std::vector<double>>& R) {
    
    // TODO: Implement noise covariance setting
}

std::vector<double> MovingHorizonEstimation::update(const std::vector<double>& z,
                                                  const std::vector<double>& u) {
    // TODO: Implement update step
    return std::vector<double>();
}

const std::vector<double>& MovingHorizonEstimation::getState() const {
    // TODO: Implement state getter
    return x_current_;
}

const std::vector<std::vector<double>>& MovingHorizonEstimation::getStateTrajectory() const {
    // TODO: Implement state trajectory getter
    return state_trajectory_;
}

void MovingHorizonEstimation::reset() {
    // TODO: Implement reset
}

std::vector<std::vector<double>> MovingHorizonEstimation::inverse(
    const std::vector<std::vector<double>>& A) const {
    
    // TODO: Implement matrix inversion
    return std::vector<std::vector<double>>();
}

std::vector<std::vector<double>> MovingHorizonEstimation::multiplyMatrices(
    const std::vector<std::vector<double>>& A,
    const std::vector<std::vector<double>>& B) const {
    
    // TODO: Implement matrix multiplication
    return std::vector<std::vector<double>>();
}

std::vector<double> MovingHorizonEstimation::multiplyMatrixVector(
    const std::vector<std::vector<double>>& A,
    const std::vector<double>& b) const {
    
    // TODO: Implement matrix-vector multiplication
    return std::vector<double>();
}
    
    // Add current measurement and control to history
    measurement_history_.push_back(z);
    control_history_.push_back(u);
    
    // Maintain fixed horizon length
    if (measurement_history_.size() > horizon_length_) {
        measurement_history_.pop_front();
        control_history_.pop_front();
    }
    
    // Create initial guess
    std::vector<std::vector<double>> initial_guess = getInitialGuess();
    
    // Call optimizer
    std::vector<std::vector<double>> optimized_trajectory;
    
    if (measurement_history_.size() == horizon_length_) {
        // Full horizon optimization
        optimized_trajectory = solver_(
            initial_guess,
            std::vector<std::vector<double>>(measurement_history_.begin(), measurement_history_.end()),
            std::vector<std::vector<double>>(control_history_.begin(), control_history_.end()),
            arrival_cost_state_,
            inverse(arrival_cost_cov_),
            Q_inv_,
            R_inv_,
            f_,
            h_
        );
    } else {
        // Partial horizon optimization (no arrival cost until full horizon)
        std::vector<double> dummy_state(state_dim_, 0.0);
        std::vector<std::vector<double>> dummy_cov_inv(state_dim_, std::vector<double>(state_dim_, 0.0));
        
        optimized_trajectory = solver_(
            initial_guess,
            std::vector<std::vector<double>>(measurement_history_.begin(), measurement_history_.end()),
            std::vector<std::vector<double>>(control_history_.begin(), control_history_.end()),
            dummy_state,
            dummy_cov_inv,
            Q_inv_,
            R_inv_,
            f_,
            h_
        );
    }
    
    // Update state trajectory
    state_trajectory_ = optimized_trajectory;
    
    // Update current state estimate (last state in the trajectory)
    x_current_ = state_trajectory_.back();
    
    // Update arrival cost for next step
    updateArrivalCost(optimized_trajectory);
    
    return x_current_;
}

const std::vector<double>& MovingHorizonEstimation::getState() const {
    if (!is_initialized_) {
        throw std::runtime_error("MHE not initialized");
    }
    return x_current_;
}

std::vector<std::vector<double>> MovingHorizonEstimation::getStateTrajectory() const {
    if (!is_initialized_) {
        throw std::runtime_error("MHE not initialized");
    }
    return state_trajectory_;
}

void MovingHorizonEstimation::reset() {
    is_initialized_ = false;
    std::fill(x_current_.begin(), x_current_.end(), 0.0);
    for (auto& state : state_trajectory_) {
        std::fill(state.begin(), state.end(), 0.0);
    }
    
    measurement_history_.clear();
    control_history_.clear();
}

void MovingHorizonEstimation::updateArrivalCost(const std::vector<std::vector<double>>& state_trajectory) {
    // Update arrival cost using EKF-like update for simplicity
    // In practice, more sophisticated methods can be used
    
    // The last state becomes the arrival cost state
    arrival_cost_state_ = state_trajectory.back();
    
    // Simple covariance update (this is a simplification)
    // In a real implementation, you'd use a more accurate arrival cost update
    for (size_t i = 0; i < state_dim_; ++i) {
        for (size_t j = 0; j < state_dim_; ++j) {
            arrival_cost_cov_[i][j] = Q_[i][j]; // Simplified
        }
    }
}

std::vector<std::vector<double>> MovingHorizonEstimation::getInitialGuess() const {
    std::vector<std::vector<double>> initial_guess;
    
    if (measurement_history_.size() < horizon_length_) {
        // Partial horizon case
        initial_guess.resize(measurement_history_.size(), std::vector<double>(state_dim_, 0.0));
        
        // Use current state estimate for all time points
        for (size_t i = 0; i < measurement_history_.size(); ++i) {
            initial_guess[i] = x_current_;
        }
    } else {
        // Full horizon case
        initial_guess = state_trajectory_;
    }
    
    return initial_guess;
}

std::vector<std::vector<double>> MovingHorizonEstimation::inverse(
    const std::vector<std::vector<double>>& A) const {
    
    size_t n = A.size();
    if (n != A[0].size()) {
        throw std::invalid_argument("Matrix must be square for inversion");
    }
    
    // Create augmented matrix [A|I]
    std::vector<std::vector<double>> aug(n, std::vector<double>(2 * n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            aug[i][j] = A[i][j];
            aug[i][j + n] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // Gauss-Jordan elimination
    for (size_t i = 0; i < n; ++i) {
        // Find pivot
        size_t pivot_row = i;
        double max_val = std::abs(aug[i][i]);
        
        for (size_t j = i + 1; j < n; ++j) {
            if (std::abs(aug[j][i]) > max_val) {
                max_val = std::abs(aug[j][i]);
                pivot_row = j;
            }
        }
        
        if (max_val < 1e-10) {
            throw std::runtime_error("Matrix is singular or poorly conditioned");
        }
        
        // Swap rows if needed
        if (pivot_row != i) {
            std::swap(aug[i], aug[pivot_row]);
        }
        
        // Normalize pivot row
        double pivot = aug[i][i];
        for (size_t j = i; j < 2 * n; ++j) {
            aug[i][j] /= pivot;
        }
        
        // Eliminate other rows
        for (size_t j = 0; j < n; ++j) {
            if (j != i) {
                double factor = aug[j][i];
                for (size_t k = i; k < 2 * n; ++k) {
                    aug[j][k] -= factor * aug[i][k];
                }
            }
        }
    }
    
    // Extract inverse matrix
    std::vector<std::vector<double>> inv(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            inv[i][j] = aug[i][j + n];
        }
    }
    
    return inv;
}

} // namespace state_estimation
