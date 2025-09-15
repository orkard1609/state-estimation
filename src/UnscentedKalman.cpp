#include "state_estimation/UnscentedKalman.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace state_estimation {

UnscentedKalmanFilter::UnscentedKalmanFilter(size_t state_dim, size_t measurement_dim,
                                           double alpha, double beta, double kappa)
    : state_dim_(state_dim),
      measurement_dim_(measurement_dim),
      sigma_point_count_(2 * state_dim + 1),
      is_initialized_(false),
      x_(state_dim, 0.0),
      P_(state_dim, std::vector<double>(state_dim, 0.0)),
      alpha_(alpha),
      beta_(beta),
      kappa_(kappa),
      weights_mean_(sigma_point_count_, 0.0),
      weights_covariance_(sigma_point_count_, 0.0) {
    
    // TODO: Implement parameter validation and weight initialization
}

void UnscentedKalmanFilter::initialize(const std::vector<double>& initial_state,
                                      const std::vector<std::vector<double>>& initial_covariance) {
    // TODO: Implement initialization
}

void UnscentedKalmanFilter::setModels(const ProcessModel& f, const MeasurementModel& h) {
    // TODO: Implement model setting
}

void UnscentedKalmanFilter::predict(const std::vector<double>& u,
                                   const std::vector<std::vector<double>>& Q) {
    // TODO: Implement prediction step
}

void UnscentedKalmanFilter::update(const std::vector<double>& z,
                                  const std::vector<std::vector<double>>& R) {
    // TODO: Implement update step
}

const std::vector<double>& UnscentedKalmanFilter::getState() const {
    // TODO: Implement state getter
    return x_;
}

const std::vector<std::vector<double>>& UnscentedKalmanFilter::getCovariance() const {
    // TODO: Implement covariance getter
    return P_;
}

void UnscentedKalmanFilter::reset() {
    // TODO: Implement reset
}

std::vector<std::vector<double>> UnscentedKalmanFilter::generateSigmaPoints() const {
    // TODO: Implement sigma point generation
    return std::vector<std::vector<double>>();
}

std::vector<double> UnscentedKalmanFilter::calculateMean(
    const std::vector<std::vector<double>>& sigma_points,
    const std::vector<double>& weights) const {
    
    // TODO: Implement mean calculation
    return std::vector<double>();
}

std::vector<std::vector<double>> UnscentedKalmanFilter::calculateCovariance(
    const std::vector<std::vector<double>>& sigma_points,
    const std::vector<double>& mean,
    const std::vector<double>& weights) const {
    
    // TODO: Implement covariance calculation
    return std::vector<std::vector<double>>();
}

std::vector<std::vector<double>> UnscentedKalmanFilter::calculateCrossCovariance(
    const std::vector<std::vector<double>>& sigma_points_x,
    const std::vector<double>& mean_x,
    const std::vector<std::vector<double>>& sigma_points_z,
    const std::vector<double>& mean_z,
    const std::vector<double>& weights) const {
    
    // TODO: Implement cross-covariance calculation
    return std::vector<std::vector<double>>();
}

std::vector<double> UnscentedKalmanFilter::subtractVectors(
    const std::vector<double>& a,
    const std::vector<double>& b) const {
    
    // TODO: Implement vector subtraction
    return std::vector<double>();
}

std::vector<std::vector<double>> UnscentedKalmanFilter::transposeMatrix(
    const std::vector<std::vector<double>>& A) const {
    
    // TODO: Implement matrix transpose
    return std::vector<std::vector<double>>();
}

std::vector<std::vector<double>> UnscentedKalmanFilter::multiplyMatrices(
    const std::vector<std::vector<double>>& A,
    const std::vector<std::vector<double>>& B) const {
    
    // TODO: Implement matrix multiplication
    return std::vector<std::vector<double>>();
}

std::vector<std::vector<double>> UnscentedKalmanFilter::invertMatrix(
    const std::vector<std::vector<double>>& A) const {
    
    // TODO: Implement matrix inversion
    return std::vector<std::vector<double>>();
}
        }
    }
}

void UnscentedKalmanFilter::update(const std::vector<double>& z,
                                 const std::vector<std::vector<double>>& R) {
    if (!is_initialized_) {
        throw std::runtime_error("UKF not initialized");
    }
    if (!h_) {
        throw std::runtime_error("Measurement model not set");
    }
    if (z.size() != measurement_dim_) {
        throw std::invalid_argument("Measurement vector dimension mismatch");
    }
    if (R.size() != measurement_dim_ || R[0].size() != measurement_dim_) {
        throw std::invalid_argument("Measurement noise covariance dimension mismatch");
    }
    
    // Generate sigma points
    std::vector<std::vector<double>> sigma_points = generateSigmaPoints();
    
    // Propagate sigma points through measurement model
    std::vector<std::vector<double>> transformed_sigma_points(sigma_point_count_,
                                                           std::vector<double>(measurement_dim_, 0.0));
    for (size_t i = 0; i < sigma_point_count_; ++i) {
        transformed_sigma_points[i] = h_(sigma_points[i]);
    }
    
    // Calculate predicted measurement mean
    std::vector<double> z_pred = calculateMean(transformed_sigma_points, weights_mean_);
    
    // Calculate measurement prediction covariance
    std::vector<std::vector<double>> S = calculateCovariance(transformed_sigma_points, z_pred, weights_covariance_);
    
    // Add measurement noise covariance
    for (size_t i = 0; i < measurement_dim_; ++i) {
        for (size_t j = 0; j < measurement_dim_; ++j) {
            S[i][j] += R[i][j];
        }
    }
    
    // Calculate cross-correlation matrix
    std::vector<std::vector<double>> cross_correlation = calculateCrossCorrelation(
        sigma_points, x_, transformed_sigma_points, z_pred, weights_covariance_);
    
    // Calculate Kalman gain
    std::vector<std::vector<double>> S_inv = inverse(S);
    std::vector<std::vector<double>> K = multiplyMatrices(cross_correlation, S_inv);
    
    // Calculate measurement residual
    std::vector<double> y = subtractVectors(z, z_pred);
    
    // Update state
    std::vector<double> Ky = multiplyMatrixVector(K, y);
    x_ = addVectors(x_, Ky);
    
    // Update covariance
    std::vector<std::vector<double>> K_S = multiplyMatrices(K, S);
    std::vector<std::vector<double>> K_S_KT = multiplyMatrices(K_S, transpose(K));
    
    for (size_t i = 0; i < state_dim_; ++i) {
        for (size_t j = 0; j < state_dim_; ++j) {
            P_[i][j] -= K_S_KT[i][j];
        }
    }
}

const std::vector<double>& UnscentedKalmanFilter::getState() const {
    if (!is_initialized_) {
        throw std::runtime_error("UKF not initialized");
    }
    return x_;
}

const std::vector<std::vector<double>>& UnscentedKalmanFilter::getCovariance() const {
    if (!is_initialized_) {
        throw std::runtime_error("UKF not initialized");
    }
    return P_;
}

void UnscentedKalmanFilter::reset() {
    is_initialized_ = false;
    std::fill(x_.begin(), x_.end(), 0.0);
    for (auto& row : P_) {
        std::fill(row.begin(), row.end(), 0.0);
    }
}

void UnscentedKalmanFilter::setParameters(double alpha, double beta, double kappa) {
    if (alpha <= 0.0 || alpha > 1.0) {
        throw std::invalid_argument("Alpha should be in range (0,1]");
    }
    
    alpha_ = alpha;
    beta_ = beta;
    kappa_ = kappa;
    
    // Recalculate lambda and weights
    lambda_ = alpha * alpha * (state_dim_ + kappa) - state_dim_;
    
    double weight_0_mean = lambda_ / (state_dim_ + lambda_);
    double weight_0_cov = weight_0_mean + (1 - alpha * alpha + beta);
    double weight_i = 1.0 / (2 * (state_dim_ + lambda_));
    
    weights_mean_[0] = weight_0_mean;
    weights_covariance_[0] = weight_0_cov;
    
    for (size_t i = 1; i < sigma_point_count_; ++i) {
        weights_mean_[i] = weight_i;
        weights_covariance_[i] = weight_i;
    }
}

std::vector<std::vector<double>> UnscentedKalmanFilter::generateSigmaPoints() const {
    std::vector<std::vector<double>> sigma_points(sigma_point_count_,
                                              std::vector<double>(state_dim_, 0.0));
    
    // First sigma point is the mean
    sigma_points[0] = x_;
    
    // Calculate square root of (n + lambda) * P
    std::vector<std::vector<double>> scaled_P(state_dim_, std::vector<double>(state_dim_, 0.0));
    for (size_t i = 0; i < state_dim_; ++i) {
        for (size_t j = 0; j < state_dim_; ++j) {
            scaled_P[i][j] = (state_dim_ + lambda_) * P_[i][j];
        }
    }
    
    std::vector<std::vector<double>> L = matrixSqrt(scaled_P);
    
    // Generate remaining sigma points
    for (size_t i = 0; i < state_dim_; ++i) {
        // x + column i of L
        for (size_t j = 0; j < state_dim_; ++j) {
            sigma_points[i + 1][j] = x_[j] + L[j][i];
        }
        
        // x - column i of L
        for (size_t j = 0; j < state_dim_; ++j) {
            sigma_points[i + state_dim_ + 1][j] = x_[j] - L[j][i];
        }
    }
    
    return sigma_points;
}

std::vector<std::vector<double>> UnscentedKalmanFilter::matrixSqrt(
    const std::vector<std::vector<double>>& A) const {
    
    // For simplicity, we use a Cholesky decomposition for positive definite matrices
    // In practice, you should use a more robust numerical method or library
    size_t n = A.size();
    std::vector<std::vector<double>> L(n, std::vector<double>(n, 0.0));
    
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            double sum = 0.0;
            
            if (j == i) {
                for (size_t k = 0; k < j; ++k) {
                    sum += L[j][k] * L[j][k];
                }
                L[j][j] = std::sqrt(A[j][j] - sum);
            } else {
                for (size_t k = 0; k < j; ++k) {
                    sum += L[i][k] * L[j][k];
                }
                L[i][j] = (A[i][j] - sum) / L[j][j];
            }
        }
    }
    
    return L;
}

std::vector<double> UnscentedKalmanFilter::calculateMean(
    const std::vector<std::vector<double>>& sigma_points,
    const std::vector<double>& weights) const {
    
    size_t dim = sigma_points[0].size();
    std::vector<double> mean(dim, 0.0);
    
    for (size_t i = 0; i < sigma_point_count_; ++i) {
        for (size_t j = 0; j < dim; ++j) {
            mean[j] += weights[i] * sigma_points[i][j];
        }
    }
    
    return mean;
}

std::vector<std::vector<double>> UnscentedKalmanFilter::calculateCovariance(
    const std::vector<std::vector<double>>& sigma_points,
    const std::vector<double>& mean,
    const std::vector<double>& weights) const {
    
    size_t dim = sigma_points[0].size();
    std::vector<std::vector<double>> cov(dim, std::vector<double>(dim, 0.0));
    
    for (size_t i = 0; i < sigma_point_count_; ++i) {
        std::vector<double> diff = subtractVectors(sigma_points[i], mean);
        
        for (size_t j = 0; j < dim; ++j) {
            for (size_t k = 0; k < dim; ++k) {
                cov[j][k] += weights[i] * diff[j] * diff[k];
            }
        }
    }
    
    return cov;
}

std::vector<std::vector<double>> UnscentedKalmanFilter::calculateCrossCorrelation(
    const std::vector<std::vector<double>>& sigma_points_a,
    const std::vector<double>& mean_a,
    const std::vector<std::vector<double>>& sigma_points_b,
    const std::vector<double>& mean_b,
    const std::vector<double>& weights) const {
    
    size_t dim_a = sigma_points_a[0].size();
    size_t dim_b = sigma_points_b[0].size();
    std::vector<std::vector<double>> cross_corr(dim_a, std::vector<double>(dim_b, 0.0));
    
    for (size_t i = 0; i < sigma_point_count_; ++i) {
        std::vector<double> diff_a = subtractVectors(sigma_points_a[i], mean_a);
        std::vector<double> diff_b = subtractVectors(sigma_points_b[i], mean_b);
        
        for (size_t j = 0; j < dim_a; ++j) {
            for (size_t k = 0; k < dim_b; ++k) {
                cross_corr[j][k] += weights[i] * diff_a[j] * diff_b[k];
            }
        }
    }
    
    return cross_corr;
}

// Matrix operations - similar to those in KalmanFilter and ExtendedKalmanFilter
std::vector<double> UnscentedKalmanFilter::multiplyMatrixVector(
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

std::vector<std::vector<double>> UnscentedKalmanFilter::multiplyMatrices(
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

std::vector<std::vector<double>> UnscentedKalmanFilter::transpose(
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

std::vector<std::vector<double>> UnscentedKalmanFilter::inverse(
    const std::vector<std::vector<double>>& A) const {
    
    // Implementation is the same as in ExtendedKalmanFilter::inverse
    // Gauss-Jordan elimination for matrix inversion
    
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

std::vector<double> UnscentedKalmanFilter::subtractVectors(
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

std::vector<double> UnscentedKalmanFilter::addVectors(
    const std::vector<double>& a, 
    const std::vector<double>& b) const {
    
    if (a.size() != b.size()) {
        throw std::invalid_argument("Vector dimensions mismatch");
    }
    
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    
    return result;
}

std::vector<std::vector<double>> UnscentedKalmanFilter::addMatrices(
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
