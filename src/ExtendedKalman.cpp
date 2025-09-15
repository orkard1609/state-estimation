#include "state_estimation/ExtendedKalman.hpp"
#include <stdexcept>
#include <cmath>

namespace state_estimation {

ExtendedKalmanFilter::ExtendedKalmanFilter(size_t state_dim, size_t measurement_dim)
    : state_dim_(state_dim),
      measurement_dim_(measurement_dim),
      is_initialized_(false),
      x_(state_dim, 0.0),
      P_(state_dim, std::vector<double>(state_dim, 0.0)) {
    
    // TODO: Implement parameter validation
}

void ExtendedKalmanFilter::initialize(const std::vector<double>& initial_state,
                                     const std::vector<std::vector<double>>& initial_covariance) {
    // TODO: Implement initialization
}

void ExtendedKalmanFilter::setModels(const ProcessModel& f, const MeasurementModel& h,
                                    const ProcessJacobian& F, const MeasurementJacobian& H) {
    // TODO: Implement model setting
}

void ExtendedKalmanFilter::predict(const std::vector<double>& u,
                                  const std::vector<std::vector<double>>& Q) {
    // TODO: Implement prediction step
}

void ExtendedKalmanFilter::update(const std::vector<double>& z,
                                 const std::vector<std::vector<double>>& R) {
    // TODO: Implement update step
}

const std::vector<double>& ExtendedKalmanFilter::getState() const {
    // TODO: Implement state getter
    return x_;
}

const std::vector<std::vector<double>>& ExtendedKalmanFilter::getCovariance() const {
    // TODO: Implement covariance getter
    return P_;
}

void ExtendedKalmanFilter::reset() {
    // TODO: Implement reset
}

std::vector<double> ExtendedKalmanFilter::multiplyMatrixVector(
    const std::vector<std::vector<double>>& A, 
    const std::vector<double>& b) const {
    
    // TODO: Implement matrix-vector multiplication
    return std::vector<double>();
}

std::vector<std::vector<double>> ExtendedKalmanFilter::multiplyMatrices(
    const std::vector<std::vector<double>>& A, 
    const std::vector<std::vector<double>>& B) const {
    
    // TODO: Implement matrix-matrix multiplication
    return std::vector<std::vector<double>>();
}

std::vector<std::vector<double>> ExtendedKalmanFilter::transpose(
    const std::vector<std::vector<double>>& A) const {
    
    // TODO: Implement matrix transpose
    return std::vector<std::vector<double>>();
}

std::vector<std::vector<double>> ExtendedKalmanFilter::inverse(
    const std::vector<std::vector<double>>& A) const {
    
    // TODO: Implement matrix inversion
    return std::vector<std::vector<double>>();
}

std::vector<double> ExtendedKalmanFilter::subtractVectors(
    const std::vector<double>& a,
    const std::vector<double>& b) const {
    
    // TODO: Implement vector subtraction
    return std::vector<double>();
}
    std::vector<std::vector<double>> H_T = transpose(H);
    std::vector<std::vector<double>> H_P_HT = multiplyMatrices(H_P, H_T);
    
    std::vector<std::vector<double>> S = addMatrices(H_P_HT, R);
    
    // Calculate Kalman gain: K = P * H^T * S^(-1)
    std::vector<std::vector<double>> S_inv = inverse(S);
    std::vector<std::vector<double>> P_HT = multiplyMatrices(P_, H_T);
    std::vector<std::vector<double>> K = multiplyMatrices(P_HT, S_inv);
    
    // Update state: x = x + K * y
    std::vector<double> Ky = multiplyMatrixVector(K, y);
    for (size_t i = 0; i < state_dim_; ++i) {
        x_[i] += Ky[i];
    }
    
    // Update covariance: P = (I - K * H) * P
    std::vector<std::vector<double>> K_H = multiplyMatrices(K, H);
    std::vector<std::vector<double>> I_KH(state_dim_, std::vector<double>(state_dim_));
    
    for (size_t i = 0; i < state_dim_; ++i) {
        for (size_t j = 0; j < state_dim_; ++j) {
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - K_H[i][j];
        }
    }
    
    P_ = multiplyMatrices(I_KH, P_);
}

const std::vector<double>& ExtendedKalmanFilter::getState() const {
    if (!is_initialized_) {
        throw std::runtime_error("EKF not initialized");
    }
    return x_;
}

const std::vector<std::vector<double>>& ExtendedKalmanFilter::getCovariance() const {
    if (!is_initialized_) {
        throw std::runtime_error("EKF not initialized");
    }
    return P_;
}

void ExtendedKalmanFilter::reset() {
    is_initialized_ = false;
    std::fill(x_.begin(), x_.end(), 0.0);
    for (auto& row : P_) {
        std::fill(row.begin(), row.end(), 0.0);
    }
}

// Matrix operations - similar to those in KalmanFilter
std::vector<double> ExtendedKalmanFilter::multiplyMatrixVector(
    const std::vector<std::vector<double>>& A, 
    const std::vector<double>& b) const {
    
    size_t m = A.size();
    size_t n = A[0].size();
    
    if (b.size() != n) {
        throw std::invalid_argument("Matrix-vector dimensions mismatch");
    }
    
    std::vector<double> result(m, 0.0);
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            result[i] += A[i][j] * b[j];
        }
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::multiplyMatrices(
    const std::vector<std::vector<double>>& A, 
    const std::vector<std::vector<double>>& B) const {
    
    size_t m = A.size();
    size_t n = A[0].size();
    size_t p = B[0].size();
    
    if (B.size() != n) {
        throw std::invalid_argument("Matrix dimensions mismatch");
    }
    
    std::vector<std::vector<double>> result(m, std::vector<double>(p, 0.0));
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < p; ++j) {
            for (size_t k = 0; k < n; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::transpose(
    const std::vector<std::vector<double>>& A) const {
    
    size_t m = A.size();
    size_t n = A[0].size();
    
    std::vector<std::vector<double>> result(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            result[j][i] = A[i][j];
        }
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::inverse(
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

std::vector<double> ExtendedKalmanFilter::subtractVectors(
    const std::vector<double>& a, 
    const std::vector<double>& b) const {
    
    if (a.size() != b.size()) {
        throw std::invalid_argument("Vector dimensions mismatch");
    }
    
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] - b[i];
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::addMatrices(
    const std::vector<std::vector<double>>& A, 
    const std::vector<std::vector<double>>& B) const {
    
    size_t m = A.size();
    size_t n = A[0].size();
    
    if (B.size() != m || B[0].size() != n) {
        throw std::invalid_argument("Matrix dimensions mismatch");
    }
    
    std::vector<std::vector<double>> result(m, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
    
    return result;
}

} // namespace state_estimation
