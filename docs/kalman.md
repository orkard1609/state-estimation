# Kalman Filter API Reference

## Class: KalmanFilter

### Description

The Kalman Filter is an optimal recursive estimator for linear systems with Gaussian noise. It provides a way to estimate the state of a system when some variables cannot be directly measured, by using a series of measurements over time.

### Namespace

```cpp
namespace state_estimation
```

### Constructor

```cpp
KalmanFilter(size_t state_dimension, size_t measurement_dimension);
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
    const std::vector<std::vector<double>>& initial_covariance
);
```

**Description:**
Initializes the filter with an initial state and covariance.

**Parameters:**
- `initial_state`: Vector containing the initial state estimate
- `initial_covariance`: Matrix containing the initial state covariance

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
    const std::vector<std::vector<double>>& F,
    const std::vector<std::vector<double>>& Q,
    const std::vector<std::vector<double>>& B = {},
    const std::vector<double>& u = {}
);
```

**Description:**
Performs the prediction step of the Kalman filter.

**Parameters:**
- `F`: State transition matrix (state_dim x state_dim)
- `Q`: Process noise covariance matrix (state_dim x state_dim)
- `B`: Control input matrix (state_dim x control_dim) - optional
- `u`: Control input vector (control_dim) - optional

**Exceptions:**
- `std::invalid_argument`: If matrix dimensions don't match
- `std::runtime_error`: If the filter hasn't been initialized

#### Update

```cpp
void update(
    const std::vector<double>& z,
    const std::vector<std::vector<double>>& H,
    const std::vector<std::vector<double>>& R
);
```

**Description:**
Performs the update step of the Kalman filter using a measurement.

**Parameters:**
- `z`: Measurement vector (meas_dim)
- `H`: Measurement matrix (meas_dim x state_dim)
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
#include "state_estimation/Kalman.hpp"
#include <vector>
#include <iostream>

int main() {
    // Create Kalman filter for position-velocity tracking (2D state: [pos, vel])
    // with position measurements (1D measurement: [pos])
    state_estimation::KalmanFilter kf(2, 1);
    
    // Initialize state and covariance
    std::vector<double> initial_state = {0.0, 1.0};  // Position=0, Velocity=1
    std::vector<std::vector<double>> initial_cov = {
        {1.0, 0.0},
        {0.0, 1.0}
    };
    
    kf.initialize(initial_state, initial_cov);
    
    // Define model matrices
    double dt = 0.1;  // Time step
    
    // State transition matrix (constant velocity model)
    std::vector<std::vector<double>> F = {
        {1.0, dt},
        {0.0, 1.0}
    };
    
    // Process noise covariance
    std::vector<std::vector<double>> Q = {
        {0.01, 0.0},
        {0.0, 0.01}
    };
    
    // Measurement matrix (measuring only position)
    std::vector<std::vector<double>> H = {
        {1.0, 0.0}
    };
    
    // Measurement noise covariance
    std::vector<std::vector<double>> R = {
        {0.1}
    };
    
    // Run filter for 10 steps
    for (int i = 0; i < 10; i++) {
        // Prediction step
        kf.predict(F, Q);
        
        // Create noisy measurement (in a real application, this would come from sensors)
        // For demo, assume true position follows velocity=1
        double true_pos = i * dt;
        double measured_pos = true_pos + 0.1 * ((double)rand() / RAND_MAX - 0.5);
        
        std::vector<double> z = {measured_pos};
        
        // Update step
        kf.update(z, H, R);
        
        // Get state estimate
        auto state = kf.getState();
        
        std::cout << "Step " << i 
                  << ": Measured=" << measured_pos
                  << ", Estimated Position=" << state[0] 
                  << ", Estimated Velocity=" << state[1] << std::endl;
    }
    
    return 0;
}
```
