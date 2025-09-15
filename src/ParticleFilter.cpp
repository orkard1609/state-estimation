#include "state_estimation/ParticleFilter.hpp"
#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <random>

namespace state_estimation {

ParticleFilter::ParticleFilter(size_t state_dim, size_t num_particles, double resample_threshold)
    : state_dim_(state_dim),
      num_particles_(num_particles),
      resample_threshold_(resample_threshold),
      is_initialized_(false),
      particles_(num_particles, std::vector<double>(state_dim, 0.0)),
      weights_(num_particles, 1.0 / num_particles),
      rng_(std::random_device{}()) {
    
    // TODO: Implement parameter validation
}

void ParticleFilter::initialize(const InitializeParticles& init_func, unsigned int seed) {
    // TODO: Implement initialization with custom function
}

void ParticleFilter::initializeGaussian(const std::vector<double>& mean,
                                       const std::vector<std::vector<double>>& covariance,
                                       unsigned int seed) {
    // TODO: Implement Gaussian initialization
}

void ParticleFilter::setModels(const TransitionModel& transition_model,
                              const MeasurementLikelihood& measurement_likelihood) {
    // TODO: Implement model setting
}

void ParticleFilter::predict(const std::vector<double>& u) {
    // TODO: Implement prediction step
}

void ParticleFilter::update(const std::vector<double>& z) {
    // TODO: Implement update step
}

void ParticleFilter::resample() {
    // TODO: Implement resampling
}

double ParticleFilter::getEffectiveSampleSize() const {
    // TODO: Implement effective sample size calculation
    return 0.0;
}

std::vector<double> ParticleFilter::getStateEstimate() const {
    // TODO: Implement state estimation
    return std::vector<double>(state_dim_, 0.0);
}

std::vector<std::vector<double>> ParticleFilter::getStateCovariance() const {
    // TODO: Implement covariance estimation
    return std::vector<std::vector<double>>(state_dim_, std::vector<double>(state_dim_, 0.0));
}

const std::vector<std::vector<double>>& ParticleFilter::getParticles() const {
    // TODO: Implement particle getter
    return particles_;
}

const std::vector<double>& ParticleFilter::getWeights() const {
    // TODO: Implement weight getter
    return weights_;
}

void ParticleFilter::reset() {
    // TODO: Implement reset
}

void ParticleFilter::update(const std::vector<double>& z) {
    if (!is_initialized_) {
        throw std::runtime_error("Particle filter not initialized");
    }
    if (!measurement_likelihood_) {
        throw std::runtime_error("Measurement likelihood function not set");
    }
    
    // Update weights based on measurement likelihood
    for (size_t i = 0; i < num_particles_; ++i) {
        double likelihood = measurement_likelihood_(particles_[i], z);
        weights_[i] *= likelihood;
    }
    
    // Normalize weights
    normalizeWeights();
    
    // Check if resampling is needed
    double effective_sample_size = calculateEffectiveSampleSize();
    if (effective_sample_size < resample_threshold_ * num_particles_) {
        resample();
    }
}

std::vector<double> ParticleFilter::getEstimatedState() const {
    if (!is_initialized_) {
        throw std::runtime_error("Particle filter not initialized");
    }
    
    // Calculate weighted mean of particles
    std::vector<double> estimated_state(state_dim_, 0.0);
    for (size_t i = 0; i < num_particles_; ++i) {
        for (size_t j = 0; j < state_dim_; ++j) {
            estimated_state[j] += weights_[i] * particles_[i][j];
        }
    }
    
    return estimated_state;
}

const std::vector<std::vector<double>>& ParticleFilter::getParticles() const {
    if (!is_initialized_) {
        throw std::runtime_error("Particle filter not initialized");
    }
    return particles_;
}

const std::vector<double>& ParticleFilter::getWeights() const {
    if (!is_initialized_) {
        throw std::runtime_error("Particle filter not initialized");
    }
    return weights_;
}

void ParticleFilter::reset() {
    is_initialized_ = false;
    for (auto& particle : particles_) {
        std::fill(particle.begin(), particle.end(), 0.0);
    }
    std::fill(weights_.begin(), weights_.end(), 1.0 / num_particles_);
}

void ParticleFilter::setNumParticles(size_t num_particles) {
    if (num_particles == 0) {
        throw std::invalid_argument("Number of particles must be greater than zero");
    }
    
    num_particles_ = num_particles;
    particles_.resize(num_particles, std::vector<double>(state_dim_, 0.0));
    weights_.resize(num_particles, 1.0 / num_particles);
    is_initialized_ = false;  // Require re-initialization with new particle count
}

void ParticleFilter::setResampleThreshold(double threshold) {
    if (threshold <= 0.0 || threshold > 1.0) {
        throw std::invalid_argument("Resample threshold must be in range (0,1]");
    }
    resample_threshold_ = threshold;
}

void ParticleFilter::resample() {
    // Implement systematic resampling
    std::vector<std::vector<double>> resampled_particles(num_particles_,
                                                       std::vector<double>(state_dim_, 0.0));
    
    // Create cumulative sum of weights
    std::vector<double> cumulative_sum(num_particles_);
    cumulative_sum[0] = weights_[0];
    for (size_t i = 1; i < num_particles_; ++i) {
        cumulative_sum[i] = cumulative_sum[i - 1] + weights_[i];
    }
    
    // Draw starting point
    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    double u = dist(rng_);
    
    // Resample
    size_t j = 0;
    for (size_t i = 0; i < num_particles_; ++i) {
        while (u > cumulative_sum[j]) {
            j++;
        }
        resampled_particles[i] = particles_[j];
        u += 1.0 / num_particles_;
    }
    
    // Update particles
    particles_ = std::move(resampled_particles);
    
    // Reset weights to uniform
    std::fill(weights_.begin(), weights_.end(), 1.0 / num_particles_);
}

double ParticleFilter::calculateEffectiveSampleSize() const {
    double sum_squared_weights = 0.0;
    for (size_t i = 0; i < num_particles_; ++i) {
        sum_squared_weights += weights_[i] * weights_[i];
    }
    
    return 1.0 / sum_squared_weights;
}

void ParticleFilter::normalizeWeights() {
    double sum = std::accumulate(weights_.begin(), weights_.end(), 0.0);
    
    if (sum < 1e-10) {
        // If all weights are very small, reset to uniform
        std::fill(weights_.begin(), weights_.end(), 1.0 / num_particles_);
    } else {
        // Normalize weights
        for (size_t i = 0; i < num_particles_; ++i) {
            weights_[i] /= sum;
        }
    }
}

} // namespace state_estimation
