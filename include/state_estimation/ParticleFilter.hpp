#ifndef STATE_ESTIMATION_PARTICLE_FILTER_HPP
#define STATE_ESTIMATION_PARTICLE_FILTER_HPP

#include <vector>
#include <functional>
#include <random>

namespace state_estimation {

/**
 * @brief Particle Filter implementation for nonlinear/non-Gaussian systems
 * 
 * Particle filter is a sequential Monte Carlo method that uses a set of particles
 * (samples) to represent the posterior distribution of the state.
 */
class ParticleFilter {
public:
    /**
     * @brief Function type for state transition model (with noise): x_{k+1} = f(x_k, u_k, w_k)
     */
    using TransitionModel = std::function<std::vector<double>(
        const std::vector<double>&, const std::vector<double>&, std::mt19937&)>;
    
    /**
     * @brief Function type for measurement likelihood: p(z_k|x_k)
     */
    using MeasurementLikelihood = std::function<double(
        const std::vector<double>&, const std::vector<double>&)>;
    
    /**
     * @brief Function type for particle initialization
     */
    using InitializeParticles = std::function<std::vector<std::vector<double>>(
        size_t, std::mt19937&)>;

    /**
     * @brief Construct a new Particle Filter
     * 
     * @param state_dim Dimension of the state vector
     * @param num_particles Number of particles
     * @param resample_threshold Effective sample size threshold fraction for resampling
     */
    ParticleFilter(size_t state_dim, size_t num_particles, double resample_threshold = 0.5);
    
    /**
     * @brief Initialize the particle filter with custom particle initializer
     * 
     * @param init_func Function to initialize particles
     * @param seed Random seed (optional)
     */
    void initialize(const InitializeParticles& init_func, unsigned int seed = 0);
    
    /**
     * @brief Initialize the particle filter with Gaussian distribution
     * 
     * @param mean Mean of the initial state
     * @param covariance Covariance of the initial state
     * @param seed Random seed (optional)
     */
    void initializeGaussian(const std::vector<double>& mean, 
                           const std::vector<std::vector<double>>& covariance,
                           unsigned int seed = 0);
    
    /**
     * @brief Set the models for the filter
     * 
     * @param transition_model State transition model function
     * @param measurement_likelihood Measurement likelihood function
     */
    void setModels(const TransitionModel& transition_model, 
                  const MeasurementLikelihood& measurement_likelihood);
    
    /**
     * @brief Prediction step
     * 
     * @param u Control input vector
     */
    void predict(const std::vector<double>& u = {});
    
    /**
     * @brief Update step
     * 
     * @param z Measurement vector
     */
    void update(const std::vector<double>& z);
    
    /**
     * @brief Get the estimated state (weighted mean of particles)
     * 
     * @return std::vector<double> Estimated state
     */
    std::vector<double> getEstimatedState() const;
    
    /**
     * @brief Get the particles
     * 
     * @return const std::vector<std::vector<double>>& Reference to particles
     */
    const std::vector<std::vector<double>>& getParticles() const;
    
    /**
     * @brief Get the weights of the particles
     * 
     * @return const std::vector<double>& Reference to weights
     */
    const std::vector<double>& getWeights() const;
    
    /**
     * @brief Reset the filter to uninitialized state
     */
    void reset();
    
    /**
     * @brief Set the number of particles
     * 
     * @param num_particles New number of particles
     */
    void setNumParticles(size_t num_particles);
    
    /**
     * @brief Set the resampling threshold
     * 
     * @param threshold Effective sample size threshold fraction for resampling
     */
    void setResampleThreshold(double threshold);

private:
    size_t state_dim_;             ///< Dimension of the state vector
    size_t num_particles_;         ///< Number of particles
    double resample_threshold_;    ///< Threshold for resampling
    bool is_initialized_;          ///< Whether the filter is initialized
    
    std::vector<std::vector<double>> particles_; ///< Particles (state samples)
    std::vector<double> weights_;                ///< Particle weights
    
    TransitionModel transition_model_;           ///< State transition model
    MeasurementLikelihood measurement_likelihood_; ///< Measurement likelihood function
    
    std::mt19937 rng_;             ///< Random number generator
    
    /**
     * @brief Resample particles based on weights
     */
    void resample();
    
    /**
     * @brief Calculate the effective sample size
     * 
     * @return double Effective sample size
     */
    double calculateEffectiveSampleSize() const;
    
    /**
     * @brief Normalize the weights to sum to 1
     */
    void normalizeWeights();
};

} // namespace state_estimation

#endif // STATE_ESTIMATION_PARTICLE_FILTER_HPP
