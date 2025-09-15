#ifndef STATE_ESTIMATION_UNSCENTED_KALMAN_HPP
#define STATE_ESTIMATION_UNSCENTED_KALMAN_HPP

#include <vector>
#include <functional>

namespace state_estimation {

/**
 * @brief Unscented Kalman Filter implementation for nonlinear systems
 * 
 * The Unscented Kalman Filter (UKF) uses the unscented transform to handle
 * nonlinearities without linearization, often providing better performance
 * than the EKF for highly nonlinear systems.
 */
class UnscentedKalmanFilter {
public:
    /**
     * @brief Function type for process model: x_{k+1} = f(x_k, u_k)
     */
    using ProcessModel = std::function<std::vector<double>(
        const std::vector<double>&, const std::vector<double>&)>;
    
    /**
     * @brief Function type for measurement model: z_k = h(x_k)
     */
    using MeasurementModel = std::function<std::vector<double>(
        const std::vector<double>&)>;

    /**
     * @brief Construct a new Unscented Kalman Filter
     * 
     * @param state_dim Dimension of the state vector
     * @param measurement_dim Dimension of the measurement vector
     * @param alpha Spread parameter for sigma points (usually 1e-3 to 1)
     * @param beta Parameter for prior knowledge of distribution (2 for Gaussian)
     * @param kappa Secondary scaling parameter (usually 0 or 3-n)
     */
    UnscentedKalmanFilter(size_t state_dim, size_t measurement_dim, 
                         double alpha = 1e-3, double beta = 2.0, double kappa = 0.0);
    
    /**
     * @brief Initialize the UKF with initial state and covariance
     * 
     * @param initial_state Initial state vector
     * @param initial_covariance Initial state covariance matrix
     */
    void initialize(const std::vector<double>& initial_state, 
                   const std::vector<std::vector<double>>& initial_covariance);
    
    /**
     * @brief Set the process and measurement models
     * 
     * @param f Process model function
     * @param h Measurement model function
     */
    void setModels(const ProcessModel& f, const MeasurementModel& h);
    
    /**
     * @brief Prediction step of the UKF
     * 
     * @param u Control input vector
     * @param Q Process noise covariance matrix
     */
    void predict(const std::vector<double>& u,
                const std::vector<std::vector<double>>& Q);
    
    /**
     * @brief Update step of the UKF
     * 
     * @param z Measurement vector
     * @param R Measurement noise covariance matrix
     */
    void update(const std::vector<double>& z,
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
    
    /**
     * @brief Set UKF parameters
     * 
     * @param alpha Spread parameter
     * @param beta Distribution parameter
     * @param kappa Secondary scaling parameter
     */
    void setParameters(double alpha, double beta, double kappa);

private:
    size_t state_dim_;         ///< Dimension of the state vector
    size_t measurement_dim_;   ///< Dimension of the measurement vector
    size_t sigma_point_count_; ///< Number of sigma points (2n+1)
    bool is_initialized_;      ///< Whether the filter is initialized
    
    std::vector<double> x_;    ///< State estimate
    std::vector<std::vector<double>> P_; ///< State covariance matrix
    
    ProcessModel f_;           ///< Process model function
    MeasurementModel h_;       ///< Measurement model function
    
    // UKF specific parameters
    double alpha_;             ///< Spread parameter
    double beta_;              ///< Distribution parameter
    double kappa_;             ///< Secondary scaling parameter
    double lambda_;            ///< Scaling factor
    std::vector<double> weights_mean_;      ///< Weights for mean calculation
    std::vector<double> weights_covariance_; ///< Weights for covariance calculation
    
    /**
     * @brief Generate sigma points from current state and covariance
     * 
     * @return std::vector<std::vector<double>> Sigma points
     */
    std::vector<std::vector<double>> generateSigmaPoints() const;
    
    /**
     * @brief Calculate the square root of a matrix using Cholesky decomposition
     * 
     * @param A Matrix to decompose
     * @return std::vector<std::vector<double>> Square root matrix
     */
    std::vector<std::vector<double>> matrixSqrt(
        const std::vector<std::vector<double>>& A) const;
    
    /**
     * @brief Calculate mean from sigma points and weights
     * 
     * @param sigma_points Sigma points
     * @param weights Weights for each sigma point
     * @return std::vector<double> Mean vector
     */
    std::vector<double> calculateMean(
        const std::vector<std::vector<double>>& sigma_points,
        const std::vector<double>& weights) const;
    
    /**
     * @brief Calculate covariance from sigma points, mean, and weights
     * 
     * @param sigma_points Sigma points
     * @param mean Mean vector
     * @param weights Weights for each sigma point
     * @return std::vector<std::vector<double>> Covariance matrix
     */
    std::vector<std::vector<double>> calculateCovariance(
        const std::vector<std::vector<double>>& sigma_points,
        const std::vector<double>& mean,
        const std::vector<double>& weights) const;
    
    /**
     * @brief Calculate cross-correlation between two sets of sigma points
     * 
     * @param sigma_points_a First set of sigma points
     * @param mean_a Mean of first set
     * @param sigma_points_b Second set of sigma points
     * @param mean_b Mean of second set
     * @param weights Weights for each sigma point
     * @return std::vector<std::vector<double>> Cross-correlation matrix
     */
    std::vector<std::vector<double>> calculateCrossCorrelation(
        const std::vector<std::vector<double>>& sigma_points_a,
        const std::vector<double>& mean_a,
        const std::vector<std::vector<double>>& sigma_points_b,
        const std::vector<double>& mean_b,
        const std::vector<double>& weights) const;
    
    // Matrix operations (similar to KalmanFilter)
    std::vector<double> multiplyMatrixVector(
        const std::vector<std::vector<double>>& A, 
        const std::vector<double>& b) const;
    
    std::vector<std::vector<double>> multiplyMatrices(
        const std::vector<std::vector<double>>& A, 
        const std::vector<std::vector<double>>& B) const;
    
    std::vector<std::vector<double>> transpose(
        const std::vector<std::vector<double>>& A) const;
    
    std::vector<std::vector<double>> inverse(
        const std::vector<std::vector<double>>& A) const;
    
    std::vector<double> subtractVectors(
        const std::vector<double>& a, 
        const std::vector<double>& b) const;
    
    std::vector<double> addVectors(
        const std::vector<double>& a, 
        const std::vector<double>& b) const;
    
    std::vector<std::vector<double>> addMatrices(
        const std::vector<std::vector<double>>& A, 
        const std::vector<std::vector<double>>& B) const;
};

} // namespace state_estimation

#endif // STATE_ESTIMATION_UNSCENTED_KALMAN_HPP
