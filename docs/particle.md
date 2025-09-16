# Particle Filter API Reference

## Class: ParticleFilter

### Description

The Particle Filter is a Monte Carlo-based method for state estimation in nonlinear systems with non-Gaussian noise. It approximates the posterior probability distribution using a set of weighted samples (particles).

### Namespace

```cpp
namespace state_estimation
```

### Constructor

```cpp
ParticleFilter(size_t state_dimension, size_t measurement_dimension, size_t num_particles);
```

**Parameters:**
- `state_dimension`: The dimension of the state vector
- `measurement_dimension`: The dimension of the measurement vector
- `num_particles`: The number of particles to use

**Exceptions:**
- `std::invalid_argument`: If state_dimension, measurement_dimension, or num_particles is 0

### Public Methods

#### InitializeGaussian

```cpp
void initializeGaussian(
    const std::vector<double>& mean,
    const std::vector<std::vector<double>>& covariance
);
```

**Description:**
Initializes the filter with particles drawn from a Gaussian distribution.

**Parameters:**
- `mean`: Vector containing the mean of the initial state distribution
- `covariance`: Matrix containing the covariance of the initial state distribution

**Exceptions:**
- `std::invalid_argument`: If dimensions don't match the filter's state dimension

#### InitializeCustom

```cpp
void initializeCustom(
    const std::vector<std::vector<double>>& particles,
    const std::vector<double>& weights
);
```

**Description:**
Initializes the filter with custom particles and weights.

**Parameters:**
- `particles`: Vector of particle state vectors
- `weights`: Vector of particle weights (should sum to 1)

**Exceptions:**
- `std::invalid_argument`: If dimensions don't match or if weights don't sum to 1 (within tolerance)

#### Reset

```cpp
void reset();
```

**Description:**
Resets the filter state, clearing all particles and making the filter uninitialized.

#### Predict

```cpp
void predict(
    std::function<std::vector<double>(const std::vector<double>&, const std::vector<double>&)> motion_model,
    const std::vector<double>& control = {}
);
```

**Description:**
Performs the prediction step by propagating all particles through the motion model.

**Parameters:**
- `motion_model`: Function that takes a state vector and control input and returns the next state
- `control`: Control input vector - optional

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

#### Update

```cpp
void update(
    std::function<double(const std::vector<double>&, const std::vector<double>&)> likelihood_function,
    const std::vector<double>& measurement
);
```

**Description:**
Performs the update step by calculating particle weights based on measurement likelihood.

**Parameters:**
- `likelihood_function`: Function that calculates the likelihood of a measurement given a state
- `measurement`: The measurement vector

**Exceptions:**
- `std::invalid_argument`: If measurement dimension doesn't match
- `std::runtime_error`: If the filter hasn't been initialized

#### Resample

```cpp
void resample();
```

**Description:**
Performs systematic resampling of particles based on their weights.

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

#### GetStateEstimate

```cpp
std::vector<double> getStateEstimate() const;
```

**Description:**
Returns the weighted mean of the particles as the state estimate.

**Returns:**
Vector containing the estimated state

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

#### GetStateCovariance

```cpp
std::vector<std::vector<double>> getStateCovariance() const;
```

**Description:**
Returns the weighted covariance of the particles.

**Returns:**
Matrix containing the estimated state covariance

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

#### GetParticles

```cpp
std::vector<std::vector<double>> getParticles() const;
```

**Description:**
Returns the current set of particles.

**Returns:**
Vector of particle state vectors

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

#### GetWeights

```cpp
std::vector<double> getWeights() const;
```

**Description:**
Returns the current particle weights.

**Returns:**
Vector of particle weights

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

#### GetEffectiveParticleCount

```cpp
double getEffectiveParticleCount() const;
```

**Description:**
Returns the effective number of particles (a measure of particle degeneracy).

**Returns:**
The effective number of particles

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

### Example Usage

```cpp
#include "state_estimation/ParticleFilter.hpp"
#include <vector>
#include <iostream>
#include <random>
#include <cmath>

int main() {
    // Create a particle filter for 1D position tracking
    // State: [position]
    // Measurement: [position with noise]
    state_estimation::ParticleFilter pf(1, 1, 1000);  // 1000 particles
    
    // Initialize with Gaussian distribution
    std::vector<double> initial_mean = {0.0};  // Start at position 0
    std::vector<std::vector<double>> initial_cov = {{1.0}};  // With variance 1
    
    pf.initializeGaussian(initial_mean, initial_cov);
    
    // Create random number generators for our simulation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> process_noise(0.0, 0.1);
    std::normal_distribution<double> measurement_noise(0.0, 0.2);
    
    // Define motion model (simple random walk)
    auto motion_model = [&gen, &process_noise](const std::vector<double>& state, const std::vector<double>& control) {
        // State evolution: position += control + noise
        std::vector<double> new_state = state;
        new_state[0] += (control.empty() ? 0.0 : control[0]) + process_noise(gen);
        return new_state;
    };
    
    // Define measurement likelihood function
    auto likelihood_fn = [&measurement_noise](const std::vector<double>& state, const std::vector<double>& measurement) {
        // Assume Gaussian measurement noise
        double diff = measurement[0] - state[0];
        return std::exp(-0.5 * std::pow(diff / 0.2, 2));  // 0.2 is the measurement std dev
    };
    
    // Run filter for 100 steps
    double true_position = 0.0;
    std::vector<double> control = {0.1};  // Move right with constant velocity
    
    for (int i = 0; i < 100; i++) {
        // Update true position
        true_position += control[0] + process_noise(gen);
        
        // Generate noisy measurement
        double measured_position = true_position + measurement_noise(gen);
        std::vector<double> measurement = {measured_position};
        
        // Prediction step
        pf.predict(motion_model, control);
        
        // Update step
        pf.update(likelihood_fn, measurement);
        
        // Force resampling every 10 steps
        if (i % 10 == 0) {
            pf.resample();
        }
        
        // Every 10 steps, print state
        if (i % 10 == 0) {
            auto state = pf.getStateEstimate();
            auto cov = pf.getStateCovariance();
            
            std::cout << "Step " << i 
                      << ": True=" << true_position
                      << ", Measured=" << measured_position
                      << ", Estimated=" << state[0]
                      << ", Variance=" << cov[0][0]
                      << ", Effective N=" << pf.getEffectiveParticleCount() << std::endl;
        }
    }
    
    return 0;
}
```
