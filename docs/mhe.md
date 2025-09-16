# Moving Horizon Estimation API Reference

## Class: MovingHorizonEstimator

### Description

Moving Horizon Estimation (MHE) is an optimization-based approach to state estimation that uses a finite window of past measurements. It allows for explicit handling of constraints and can be more robust for highly nonlinear systems compared to Kalman-based filters.

### Namespace

```cpp
namespace state_estimation
```

### Constructor

```cpp
MovingHorizonEstimator(size_t state_dimension, size_t measurement_dimension, size_t horizon_length);
```

**Parameters:**
- `state_dimension`: The dimension of the state vector
- `measurement_dimension`: The dimension of the measurement vector
- `horizon_length`: The number of past measurements to consider

**Exceptions:**
- `std::invalid_argument`: If state_dimension, measurement_dimension, or horizon_length is 0

### Public Methods

#### Initialize

```cpp
void initialize(
    const std::vector<double>& initial_state,
    const std::vector<std::vector<double>>& initial_covariance
);
```

**Description:**
Initializes the estimator with an initial state and covariance.

**Parameters:**
- `initial_state`: Vector containing the initial state estimate
- `initial_covariance`: Matrix containing the initial state covariance

**Exceptions:**
- `std::invalid_argument`: If dimensions don't match the estimator's state dimension

#### Reset

```cpp
void reset();
```

**Description:**
Resets the estimator state, clearing the measurement history and making it uninitialized.

#### Update

```cpp
void update(
    std::function<std::vector<double>(const std::vector<double>&, const std::vector<double>&)> state_function,
    std::function<std::vector<double>(const std::vector<double>&)> measurement_function,
    std::function<std::vector<std::vector<double>>(const std::vector<double>&, const std::vector<double>&)> state_jacobian,
    std::function<std::vector<std::vector<double>>(const std::vector<double>&)> measurement_jacobian,
    const std::vector<double>& measurement,
    const std::vector<double>& control,
    const std::vector<std::vector<double>>& process_noise_covariance,
    const std::vector<std::vector<double>>& measurement_noise_covariance
);
```

**Description:**
Updates the state estimate using the latest measurement and solves the MHE optimization problem.

**Parameters:**
- `state_function`: Function that computes the next state given current state and control
- `measurement_function`: Function that computes the expected measurement given the state
- `state_jacobian`: Function that computes the Jacobian of the state function with respect to the state
- `measurement_jacobian`: Function that computes the Jacobian of the measurement function with respect to the state
- `measurement`: The latest measurement vector
- `control`: The latest control input vector
- `process_noise_covariance`: The process noise covariance matrix (Q)
- `measurement_noise_covariance`: The measurement noise covariance matrix (R)

**Exceptions:**
- `std::invalid_argument`: If dimensions don't match
- `std::runtime_error`: If the estimator hasn't been initialized

#### GetStateEstimate

```cpp
std::vector<double> getStateEstimate() const;
```

**Description:**
Returns the current state estimate.

**Returns:**
Vector containing the current state estimate

**Exceptions:**
- `std::runtime_error`: If the estimator hasn't been initialized

#### GetStateCovariance

```cpp
std::vector<std::vector<double>> getStateCovariance() const;
```

**Description:**
Returns the current state covariance estimate.

**Returns:**
Matrix containing the current state covariance

**Exceptions:**
- `std::runtime_error`: If the estimator hasn't been initialized

#### GetHorizonLength

```cpp
size_t getHorizonLength() const;
```

**Description:**
Returns the horizon length used by the estimator.

**Returns:**
The horizon length (number of past measurements considered)

#### SetConstraints

```cpp
void setConstraints(
    std::function<bool(const std::vector<double>&)> state_constraint_function
);
```

**Description:**
Sets a constraint function for the state that will be enforced during estimation.

**Parameters:**
- `state_constraint_function`: Function that returns true if the state satisfies constraints, false otherwise

### Example Usage

```cpp
#include "state_estimation/MovingHorizonEstimation.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#include <random>

int main() {
    // Create MHE for a simple nonlinear system
    // State: [position, velocity]
    // Measurement: [position]
    state_estimation::MovingHorizonEstimator mhe(2, 1, 10);  // 10-step horizon
    
    // Initialize state and covariance
    std::vector<double> initial_state = {0.0, 1.0};  // Start at position 0, velocity 1
    std::vector<std::vector<double>> initial_cov = {
        {1.0, 0.0},
        {0.0, 1.0}
    };
    
    mhe.initialize(initial_state, initial_cov);
    
    // Define system model
    double dt = 0.1;  // time step
    
    // Nonlinear state transition function (position + velocity with drag)
    auto state_function = [dt](const std::vector<double>& x, const std::vector<double>& u) {
        double pos = x[0];
        double vel = x[1];
        
        // Nonlinear drag: dv/dt = -k*v² * sign(v) + u
        double drag_coef = 0.1;
        double drag = drag_coef * vel * std::abs(vel);
        double acc = (u.empty() ? 0.0 : u[0]) - drag;
        
        // Euler integration
        double new_vel = vel + acc * dt;
        double new_pos = pos + vel * dt;
        
        return std::vector<double>{new_pos, new_vel};
    };
    
    // Measurement function (measuring position)
    auto measurement_function = [](const std::vector<double>& x) {
        return std::vector<double>{x[0]};  // Only measure position
    };
    
    // State Jacobian
    auto state_jacobian = [dt](const std::vector<double>& x, const std::vector<double>& u) {
        double vel = x[1];
        
        // Compute df/dx
        double drag_coef = 0.1;
        double drag_derivative = 2.0 * drag_coef * std::abs(vel);  // d(drag)/dv
        
        return std::vector<std::vector<double>>{
            {1.0, dt},
            {0.0, 1.0 - drag_derivative * dt}
        };
    };
    
    // Measurement Jacobian
    auto measurement_jacobian = [](const std::vector<double>& x) {
        return std::vector<std::vector<double>>{
            {1.0, 0.0}  // dh/dx
        };
    };
    
    // Process and measurement noise
    std::vector<std::vector<double>> Q = {
        {0.01, 0.0},
        {0.0, 0.01}
    };
    
    std::vector<std::vector<double>> R = {
        {0.1}
    };
    
    // Random number generators for simulation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> process_noise(0.0, 0.1);
    std::normal_distribution<double> measurement_noise(0.0, 0.3);
    
    // Run estimator for 100 steps
    double true_pos = 0.0;
    double true_vel = 1.0;
    std::vector<double> control = {0.5};  // Constant acceleration
    
    for (int i = 0; i < 100; i++) {
        // Update true state
        double drag = 0.1 * true_vel * std::abs(true_vel);
        double acc = control[0] - drag + process_noise(gen);
        true_vel += acc * dt;
        true_pos += true_vel * dt;
        
        // Generate measurement
        double measured_pos = true_pos + measurement_noise(gen);
        std::vector<double> measurement = {measured_pos};
        
        // Update MHE
        mhe.update(
            state_function,
            measurement_function,
            state_jacobian,
            measurement_jacobian,
            measurement,
            control,
            Q,
            R
        );
        
        // Every 10 steps, print state
        if (i % 10 == 0) {
            auto state = mhe.getStateEstimate();
            std::cout << "Step " << i 
                      << ": True Position=" << true_pos
                      << ", True Velocity=" << true_vel
                      << ", Estimated Position=" << state[0]
                      << ", Estimated Velocity=" << state[1] << std::endl;
        }
    }
    
    return 0;
}
```
