#ifndef STATE_ESTIMATION_RLS_HPP
#define STATE_ESTIMATION_RLS_HPP

#include <vector>

namespace state_estimation {

/**
 * @brief Recursive Least Squares (RLS) adaptive filter implementation
 * 
 * RLS is an adaptive filter that recursively finds the filter coefficients
 * that minimize a weighted linear least squares cost function related to the input signals.
 * It exhibits faster convergence compared to the LMS algorithm.
 */
class RLS {
public:
    /**
     * @brief Construct a new RLS filter
     * 
     * @param filter_length Length of the filter (number of coefficients)
     * @param forgetting_factor Forgetting factor (0 < lambda <= 1)
     * @param delta Small positive constant for numerical stability of P matrix initialization
     */
    RLS(size_t filter_length, double forgetting_factor = 0.99, double delta = 0.01);

    /**
     * @brief Update the filter with a new input sample
     * 
     * @param input Current input sample
     * @param desired Desired output sample
     * @return double Filtered output
     */
    double update(double input, double desired);
    
    /**
     * @brief Update the filter with a vector of input samples
     * 
     * @param input Vector of input samples
     * @param desired Vector of desired output samples
     * @return std::vector<double> Vector of filtered outputs
     */
    std::vector<double> update(const std::vector<double>& input, 
                              const std::vector<double>& desired);
    
    /**
     * @brief Get the current filter coefficients
     * 
     * @return const std::vector<double>& Reference to the filter coefficients
     */
    const std::vector<double>& getCoefficients() const;
    
    /**
     * @brief Set the forgetting factor
     * 
     * @param forgetting_factor New forgetting factor (0 < lambda <= 1)
     */
    void setForgettingFactor(double forgetting_factor);
    
    /**
     * @brief Get the current forgetting factor
     * 
     * @return double Current forgetting factor
     */
    double getForgettingFactor() const;
    
    /**
     * @brief Reset the filter to initial state
     */
    void reset();

private:
    size_t filter_length_;             ///< Filter length
    std::vector<double> coefficients_; ///< Filter coefficients
    std::vector<double> delay_line_;   ///< Input delay line
    std::vector<std::vector<double>> P_; ///< Inverse correlation matrix
    double forgetting_factor_;         ///< Forgetting factor (lambda)
    double delta_;                     ///< Regularization parameter
    
    /**
     * @brief Update the delay line with a new input sample
     * 
     * @param input New input sample
     */
    void updateDelayLine(double input);
    
    /**
     * @brief Initialize the inverse correlation matrix
     */
    void initializeP();
};

} // namespace state_estimation

#endif // STATE_ESTIMATION_RLS_HPP
