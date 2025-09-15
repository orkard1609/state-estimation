#ifndef STATE_ESTIMATION_EXTENDED_KALMAN_HPP
#define STATE_ESTIMATION_EXTENDED_KALMAN_HPP

#include <vector>
#include <functional>

namespace state_estimation {

/**
 * @brief Extended Kalman Filter implementation for nonlinear systems
 * 
 * The Extended Kalman Filter (EKF) extends the Kalman filter to nonlinear
 * systems by linearizing about the current mean and covariance.
 */
class ExtendedKalmanFilter {
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
     * @brief Function type for process model Jacobian: F_k = df/dx|_{x=x_k}
     */
    using ProcessJacobian = std::function<std::vector<std::vector<double>>(
        const std::vector<double>&, const std::vector<double>&)>;
    
    /**
     * @brief Function type for measurement model Jacobian: H_k = dh/dx|_{x=x_k}
     */
    using MeasurementJacobian = std::function<std::vector<std::vector<double>>(
        const std::vector<double>&)>;

    /**
     * @brief Construct a new Extended Kalman Filter
     * 
     * @param state_dim Dimension of the state vector
     * @param measurement_dim Dimension of the measurement vector
     */
    ExtendedKalmanFilter(size_t state_dim, size_t measurement_dim);
    
    /**
     * @brief Initialize the EKF with initial state and covariance
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
     * @param F Process model Jacobian function
     * @param H Measurement model Jacobian function
     */
    void setModels(const ProcessModel& f, const MeasurementModel& h,
                  const ProcessJacobian& F, const MeasurementJacobian& H);
    
    /**
     * @brief Prediction step of the EKF
     * 
     * @param u Control input vector
     * @param Q Process noise covariance matrix
     */
    void predict(const std::vector<double>& u,
                const std::vector<std::vector<double>>& Q);
    
    /**
     * @brief Update step of the EKF
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

private:
    size_t state_dim_;         ///< Dimension of the state vector
    size_t measurement_dim_;   ///< Dimension of the measurement vector
    bool is_initialized_;      ///< Whether the filter is initialized
    
    std::vector<double> x_;    ///< State estimate
    std::vector<std::vector<double>> P_; ///< State covariance matrix
    
    ProcessModel f_;           ///< Process model function
    MeasurementModel h_;       ///< Measurement model function
    ProcessJacobian F_;        ///< Process model Jacobian function
    MeasurementJacobian H_;    ///< Measurement model Jacobian function
    
    // Matrix operations (same as in KalmanFilter)
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
    
    std::vector<std::vector<double>> addMatrices(
        const std::vector<std::vector<double>>& A, 
        const std::vector<std::vector<double>>& B) const;
};

} // namespace state_estimation

#endif // STATE_ESTIMATION_EXTENDED_KALMAN_HPP
