#ifndef STATE_ESTIMATION_LMS_HPP
#define STATE_ESTIMATION_LMS_HPP

#include <vector>

namespace state_estimation {

/**
 * @brief Least Mean Squares (LMS) adaptive filter implementation
 * 
 * LMS is a stochastic gradient descent-based adaptive filter that minimizes
 * the mean square error between the desired signal and filter output.
 */
class LMS {
public:
    /**
     * @brief Construct a new LMS filter
     * 
     * @param filter_length Length of the filter (number of coefficients)
     * @param learning_rate Step size parameter controlling adaptation speed
     */
    LMS(size_t filter_length, double learning_rate);

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
     * @brief Set the learning rate
     * 
     * @param learning_rate New learning rate
     */
    void setLearningRate(double learning_rate);
    
    /**
     * @brief Get the current learning rate
     * 
     * @return double Current learning rate
     */
    double getLearningRate() const;
    
    /**
     * @brief Reset the filter coefficients to zero
     */
    void reset();

private:
    std::vector<double> coefficients_; ///< Filter coefficients
    std::vector<double> delay_line_;   ///< Input delay line
    double learning_rate_;             ///< Adaptation step size
    
    /**
     * @brief Update the delay line with a new input sample
     * 
     * @param input New input sample
     */
    void updateDelayLine(double input);
};

} // namespace state_estimation

#endif // STATE_ESTIMATION_LMS_HPP
