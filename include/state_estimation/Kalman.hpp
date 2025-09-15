#ifndef STATE_ESTIMATION_KALMAN_HPP
#define STATE_ESTIMATION_KALMAN_HPP

#include <vector>

namespace state_estimation {

/**
 * @brief Kalman Filter implementation for linear systems
 * 
 * The Kalman filter is an optimal recursive estimator for linear systems
 * with Gaussian process and measurement noise.
 */
class KalmanFilter {
public:
    /**
     * @brief Construct a new Kalman Filter
     * 
     * @param state_dim Dimension of the state vector
     * @param measurement_dim Dimension of the measurement vector
     */
    KalmanFilter(size_t state_dim, size_t measurement_dim);
    
    /**
     * @brief Initialize the Kalman filter with initial state and covariance
     * 
     * @param initial_state Initial state vector
     * @param initial_covariance Initial state covariance matrix
     */
    void initialize(const std::vector<double>& initial_state, 
                   const std::vector<std::vector<double>>& initial_covariance);
    
    /**
     * @brief Prediction step of the Kalman filter
     * 
     * @param F State transition matrix
     * @param Q Process noise covariance matrix
     * @param B Control input matrix (optional)
     * @param u Control input vector (optional)
     */
    void predict(const std::vector<std::vector<double>>& F,
                const std::vector<std::vector<double>>& Q,
                const std::vector<std::vector<double>>& B = {},
                const std::vector<double>& u = {});
    
    /**
     * @brief Update step of the Kalman filter
     * 
     * @param z Measurement vector
     * @param H Measurement matrix
     * @param R Measurement noise covariance matrix
     */
    void update(const std::vector<double>& z,
               const std::vector<std::vector<double>>& H,
               const std::vector<std::vector<double>>& R);
    
    /**
     * @brief Get the current state estimate
     * 
     * @return const std::vector<double>& Reference to the current state estimate
     */
    const std::vector<double>& getState() const;
    
    /**
     * @brief Get the current state covariance matrix
     * 
     * @return const std::vector<std::vector<double>>& Reference to the current state covariance
     */
    const std::vector<std::vector<double>>& getCovariance() const;
    
    /**
     * @brief Reset the filter to uninitialized state
     */
    void reset();

private:
    size_t state_dim_;         ///< Dimension of the state vector
    size_t measurement_dim_;   ///< Dimension of the measurement vector
    bool is_initialized_;      ///< Whether the filter is initialized
    
    std::vector<double> x_;    ///< State estimate
    std::vector<std::vector<double>> P_; ///< State covariance matrix
    
    /**
     * @brief Matrix-vector multiplication
     * 
     * @param A Matrix
     * @param b Vector
     * @return std::vector<double> Result of A*b
     */
    std::vector<double> multiplyMatrixVector(
        const std::vector<std::vector<double>>& A, 
        const std::vector<double>& b) const;
    
    /**
     * @brief Matrix-matrix multiplication
     * 
     * @param A First matrix
     * @param B Second matrix
     * @return std::vector<std::vector<double>> Result of A*B
     */
    std::vector<std::vector<double>> multiplyMatrices(
        const std::vector<std::vector<double>>& A, 
        const std::vector<std::vector<double>>& B) const;
    
    /**
     * @brief Matrix transpose
     * 
     * @param A Matrix to transpose
     * @return std::vector<std::vector<double>> Transpose of A
     */
    std::vector<std::vector<double>> transpose(
        const std::vector<std::vector<double>>& A) const;
    
    /**
     * @brief Matrix inverse (for small matrices)
     * 
     * @param A Matrix to invert
     * @return std::vector<std::vector<double>> Inverse of A
     */
    std::vector<std::vector<double>> inverse(
        const std::vector<std::vector<double>>& A) const;
};

} // namespace state_estimation

#endif // STATE_ESTIMATION_KALMAN_HPP
