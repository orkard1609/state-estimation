#ifndef STATE_ESTIMATION_MOVING_HORIZON_ESTIMATION_HPP
#define STATE_ESTIMATION_MOVING_HORIZON_ESTIMATION_HPP

#include <vector>
#include <functional>
#include <deque>

namespace state_estimation {

/**
 * @brief Moving Horizon Estimation implementation
 * 
 * Moving Horizon Estimation (MHE) is an optimization-based state estimation technique
 * that estimates the state trajectory over a finite time horizon by minimizing
 * a cost function including measurement fit and prior information.
 */
class MovingHorizonEstimation {
public:
    /**
     * @brief Function type for process model: x_{k+1} = f(x_k, u_k) + w_k
     */
    using ProcessModel = std::function<std::vector<double>(
        const std::vector<double>&, const std::vector<double>&)>;
    
    /**
     * @brief Function type for measurement model: z_k = h(x_k) + v_k
     */
    using MeasurementModel = std::function<std::vector<double>(
        const std::vector<double>&)>;
    
    /**
     * @brief Function type for optimization solver
     * This function minimizes the MHE cost function given the horizon data
     */
    using OptimizationSolver = std::function<std::vector<std::vector<double>>(
        const std::vector<std::vector<double>>& initial_guess,
        const std::vector<std::vector<double>>& measurements,
        const std::vector<std::vector<double>>& controls,
        const std::vector<double>& arrival_cost_state,
        const std::vector<std::vector<double>>& arrival_cost_cov_inv,
        const std::vector<std::vector<double>>& process_noise_cov_inv,
        const std::vector<std::vector<double>>& meas_noise_cov_inv,
        const ProcessModel& f,
        const MeasurementModel& h);

    /**
     * @brief Construct a new Moving Horizon Estimation object
     * 
     * @param state_dim Dimension of the state vector
     * @param measurement_dim Dimension of the measurement vector
     * @param control_dim Dimension of the control input vector
     * @param horizon_length Length of the estimation horizon
     */
    MovingHorizonEstimation(size_t state_dim, size_t measurement_dim, 
                           size_t control_dim, size_t horizon_length);
    
    /**
     * @brief Initialize the MHE with initial state and covariance
     * 
     * @param initial_state Initial state vector
     * @param initial_covariance Initial state covariance matrix
     */
    void initialize(const std::vector<double>& initial_state, 
                   const std::vector<std::vector<double>>& initial_covariance);
    
    /**
     * @brief Set the models and solver
     * 
     * @param f Process model function
     * @param h Measurement model function
     * @param solver Optimization solver function
     */
    void setModels(const ProcessModel& f, const MeasurementModel& h,
                  const OptimizationSolver& solver);
    
    /**
     * @brief Set noise covariance matrices
     * 
     * @param Q Process noise covariance matrix
     * @param R Measurement noise covariance matrix
     */
    void setNoiseCovariances(const std::vector<std::vector<double>>& Q,
                            const std::vector<std::vector<double>>& R);
    
    /**
     * @brief Update the estimator with new measurement and control
     * 
     * @param z Measurement vector
     * @param u Control input vector
     * @return std::vector<double> Current state estimate
     */
    std::vector<double> update(const std::vector<double>& z,
                              const std::vector<double>& u);
    
    /**
     * @brief Get the current state estimate
     * 
     * @return const std::vector<double>& Reference to the current state estimate
     */
    const std::vector<double>& getState() const;
    
    /**
     * @brief Get the estimated state trajectory over the horizon
     * 
     * @return std::vector<std::vector<double>> State trajectory
     */
    std::vector<std::vector<double>> getStateTrajectory() const;
    
    /**
     * @brief Reset the estimator to uninitialized state
     */
    void reset();

private:
    size_t state_dim_;         ///< Dimension of the state vector
    size_t measurement_dim_;   ///< Dimension of the measurement vector
    size_t control_dim_;       ///< Dimension of the control input vector
    size_t horizon_length_;    ///< Length of the estimation horizon
    bool is_initialized_;      ///< Whether the estimator is initialized
    
    std::vector<double> x_current_;  ///< Current state estimate
    std::vector<std::vector<double>> state_trajectory_; ///< Estimated state trajectory over horizon
    
    std::deque<std::vector<double>> measurement_history_; ///< History of measurements over horizon
    std::deque<std::vector<double>> control_history_;     ///< History of controls over horizon
    
    std::vector<double> arrival_cost_state_; ///< State for arrival cost (prior)
    std::vector<std::vector<double>> arrival_cost_cov_; ///< Covariance for arrival cost (prior)
    
    std::vector<std::vector<double>> Q_; ///< Process noise covariance matrix
    std::vector<std::vector<double>> R_; ///< Measurement noise covariance matrix
    std::vector<std::vector<double>> Q_inv_; ///< Inverse of process noise covariance
    std::vector<std::vector<double>> R_inv_; ///< Inverse of measurement noise covariance
    
    ProcessModel f_;           ///< Process model function
    MeasurementModel h_;       ///< Measurement model function
    OptimizationSolver solver_; ///< Optimization solver
    
    /**
     * @brief Update the arrival cost (prior information for next step)
     * 
     * @param state_trajectory Estimated state trajectory
     */
    void updateArrivalCost(const std::vector<std::vector<double>>& state_trajectory);
    
    /**
     * @brief Get the initial guess for the optimization
     * 
     * @return std::vector<std::vector<double>> Initial guess for state trajectory
     */
    std::vector<std::vector<double>> getInitialGuess() const;
    
    /**
     * @brief Calculate the inverse of a matrix
     * 
     * @param A Matrix to invert
     * @return std::vector<std::vector<double>> Inverse of A
     */
    std::vector<std::vector<double>> inverse(
        const std::vector<std::vector<double>>& A) const;
};

} // namespace state_estimation

#endif // STATE_ESTIMATION_MOVING_HORIZON_ESTIMATION_HPP
