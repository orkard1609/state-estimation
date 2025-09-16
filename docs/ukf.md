# Unscented Kalman Filter API Reference

## Class: UnscentedKalmanFilter

### Description

The Unscented Kalman Filter (UKF) handles nonlinear systems by using a deterministic sampling approach called the unscented transform to propagate the state and covariance through nonlinear transformations. It often provides better accuracy than the Extended Kalman Filter for highly nonlinear systems.

### Namespace

```cpp
namespace state_estimation
```

### Constructor

```cpp
UnscentedKalmanFilter(size_t state_dimension, size_t measurement_dimension);
```

**Parameters:**
- `state_dimension`: The dimension of the state vector
- `measurement_dimension`: The dimension of the measurement vector

**Exceptions:**
- `std::invalid_argument`: If state_dimension or measurement_dimension is 0

### Public Methods

#### Initialize

```cpp
void initialize(
    const std::vector<double>& initial_state,
    const std::vector<std::vector<double>>& initial_covariance,
    double alpha = 1e-3,
    double beta = 2.0,
    double kappa = 0.0
);
```

**Description:**
Initializes the filter with an initial state and covariance, and optionally sets the sigma point parameters.

**Parameters:**
- `initial_state`: Vector containing the initial state estimate
- `initial_covariance`: Matrix containing the initial state covariance
- `alpha`: Controls the spread of sigma points (typically a small positive value)
- `beta`: Incorporates prior knowledge of the distribution (2 is optimal for Gaussian)
- `kappa`: Secondary scaling parameter (typically 0 or 3-n for n=state_dimension)

**Exceptions:**
- `std::invalid_argument`: If dimensions don't match the filter's state dimension

#### Reset

```cpp
void reset();
```

**Description:**
Resets the filter state, clearing all internal variables and making the filter uninitialized.

#### Predict

```cpp
void predict(
    std::function<std::vector<double>(const std::vector<double>&, const std::vector<double>&)> state_function,
    const std::vector<std::vector<double>>& Q,
    const std::vector<double>& u = {}
);
```

**Description:**
Performs the prediction step of the Unscented Kalman filter using a nonlinear state transition function.

**Parameters:**
- `state_function`: Nonlinear state transition function f(x, u) that takes state and control input and returns next state
- `Q`: Process noise covariance matrix (state_dim x state_dim)
- `u`: Control input vector - optional

**Exceptions:**
- `std::invalid_argument`: If matrix dimensions don't match
- `std::runtime_error`: If the filter hasn't been initialized

#### Update

```cpp
void update(
    std::function<std::vector<double>(const std::vector<double>&)> measurement_function,
    const std::vector<double>& z,
    const std::vector<std::vector<double>>& R
);
```

**Description:**
Performs the update step of the Unscented Kalman filter using a nonlinear measurement function.

**Parameters:**
- `measurement_function`: Nonlinear measurement function h(x) that takes state and returns predicted measurement
- `z`: Measurement vector (meas_dim)
- `R`: Measurement noise covariance matrix (meas_dim x meas_dim)

**Exceptions:**
- `std::invalid_argument`: If matrix dimensions don't match
- `std::runtime_error`: If the filter hasn't been initialized

#### GetState

```cpp
std::vector<double> getState() const;
```

**Description:**
Returns the current state estimate.

**Returns:**
Vector containing the current state estimate

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

#### GetCovariance

```cpp
std::vector<std::vector<double>> getCovariance() const;
```

**Description:**
Returns the current state covariance matrix.

**Returns:**
Matrix containing the current state covariance

**Exceptions:**
- `std::runtime_error`: If the filter hasn't been initialized

### Example Usage

```cpp
#include "state_estimation/UnscentedKalman.hpp"
#include <vector>
#include <iostream>
#include <cmath>

int main() {
    // Create UKF for a pendulum system
    // State: [angle, angular_velocity]
    state_estimation::UnscentedKalmanFilter ukf(2, 1);
    
    // Initialize with small angle, zero velocity
    std::vector<double> initial_state = {0.1, 0.0};
    std::vector<std::vector<double>> initial_cov = {
        {0.01, 0.0},
        {0.0, 0.01}
    };
    
    // Initialize with custom sigma point parameters
    ukf.initialize(initial_state, initial_cov, 1e-3, 2.0, 0.0);
    
    // Define pendulum nonlinear dynamics
    auto pendulum_dynamics = [](const std::vector<double>& x, const std::vector<double>& u) {
        // Simple pendulum: d²θ/dt² = -g/L * sin(θ)
        const double dt = 0.1;  // time step
        const double g = 9.81;  // gravity
        const double L = 1.0;   // pendulum length
        
        double angle = x[0];
        double angular_vel = x[1];
        
        // Euler integration
        double new_angular_vel = angular_vel - (g/L) * sin(angle) * dt;
        double new_angle = angle + angular_vel * dt;
        
        return std::vector<double>{new_angle, new_angular_vel};
    };
    
    // Define measurement function (measuring horizontal position)
    auto measurement_function = [](const std::vector<double>& x) {
        // h(x) = L * sin(θ)
        const double L = 1.0;
        return std::vector<double>{L * sin(x[0])};
    };
    
    // Process and measurement noise
    std::vector<std::vector<double>> Q = {
        {0.001, 0.0},
        {0.0, 0.001}
    };
    
    std::vector<std::vector<double>> R = {
        {0.01}
    };
    
    // Run filter for 100 steps
    std::vector<double> u;  // No control input
    
    for (int i = 0; i < 100; i++) {
        // Prediction step
        ukf.predict(pendulum_dynamics, Q, u);
        
        // Get true state (for simulation)
        auto true_state = ukf.getState();
        
        // Create simulated measurement with noise
        auto true_measurement = measurement_function(true_state);
        auto noisy_measurement = true_measurement;
        noisy_measurement[0] += 0.05 * ((double)rand() / RAND_MAX - 0.5);
        
        // Update step
        ukf.update(measurement_function, noisy_measurement, R);
        
        // Every 10 steps, print state
        if (i % 10 == 0) {
            auto state = ukf.getState();
            std::cout << "Step " << i 
                      << ": Angle=" << state[0] 
                      << ", Angular Velocity=" << state[1] << std::endl;
        }
    }
    
    return 0;
}
```
