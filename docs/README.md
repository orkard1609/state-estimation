# State Estimation Library Documentation

## Overview

This library provides a set of state estimation algorithms commonly used in signal processing, control systems, and robotics. The library is implemented in C++ and includes the following algorithms:

1. Least Mean Squares (LMS) - Adaptive filtering algorithm
2. Recursive Least Squares (RLS) - Adaptive filtering algorithm
3. Kalman Filter - Optimal state estimation for linear systems
4. Extended Kalman Filter (EKF) - Nonlinear state estimation using linearization
5. Unscented Kalman Filter (UKF) - Nonlinear state estimation using sigma points
6. Particle Filter - Monte Carlo based nonlinear state estimation
7. Moving Horizon Estimation (MHE) - Optimization-based state estimation with constraints

## Installation

### Prerequisites

- C++17 compatible compiler
- CMake 3.10 or higher
- Catch2 (for running tests)

### Building the Library

```bash
mkdir build
cd build
cmake ..
make
```

### Running the Tests

```bash
cd build
make test
```

## Getting Started

Here's a simple example using the Kalman filter to track a moving object:

```cpp
#include "state_estimation/Kalman.hpp"
#include <vector>

int main() {
    // Create a Kalman filter for a 2D position+velocity state (4D state vector)
    // and 2D position measurements
    state_estimation::KalmanFilter kf(4, 2);
    
    // Initialize state [x, y, vx, vy]
    std::vector<double> initial_state = {0.0, 0.0, 1.0, 1.0};
    
    // Initialize state covariance
    std::vector<std::vector<double>> initial_cov = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    kf.initialize(initial_state, initial_cov);
    
    // Define constant velocity model
    double dt = 0.1;
    std::vector<std::vector<double>> F = {
        {1.0, 0.0, dt, 0.0},
        {0.0, 1.0, 0.0, dt},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    // Define process noise
    std::vector<std::vector<double>> Q = {
        {0.01, 0.0, 0.0, 0.0},
        {0.0, 0.01, 0.0, 0.0},
        {0.0, 0.0, 0.01, 0.0},
        {0.0, 0.0, 0.0, 0.01}
    };
    
    // Define measurement matrix
    std::vector<std::vector<double>> H = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0}
    };
    
    // Define measurement noise
    std::vector<std::vector<double>> R = {
        {0.1, 0.0},
        {0.0, 0.1}
    };
    
    // Run filter for 10 steps
    for (int i = 0; i < 10; i++) {
        // Predict
        kf.predict(F, Q);
        
        // Generate measurement (in a real application, this would come from sensors)
        std::vector<double> measurement = {
            /* x position */ i * dt + 0.1 * (rand() / RAND_MAX - 0.5),
            /* y position */ i * dt + 0.1 * (rand() / RAND_MAX - 0.5)
        };
        
        // Update
        kf.update(measurement, H, R);
        
        // Get current state estimate
        auto state = kf.getState();
        
        // Print position
        std::cout << "Position: (" << state[0] << ", " << state[1] << ")" << std::endl;
    }
    
    return 0;
}
```

## Algorithm Descriptions

### Least Mean Squares (LMS)

The LMS algorithm is an adaptive filter that updates its coefficients using the gradient descent method to minimize the mean squared error between the output and a desired signal.

### Recursive Least Squares (RLS)

The RLS algorithm is another adaptive filter that converges faster than LMS by using a recursive approach to minimizing the weighted least squares cost function.

### Kalman Filter

The Kalman filter is an optimal recursive estimator for linear systems with Gaussian noise. It operates in a prediction-correction cycle to estimate the state of a system.

### Extended Kalman Filter (EKF)

The Extended Kalman Filter extends the Kalman filter to nonlinear systems by linearizing the system model around the current state estimate.

### Unscented Kalman Filter (UKF)

The Unscented Kalman Filter handles nonlinear systems by using a deterministic sampling approach called the unscented transform to capture mean and covariance information.

### Particle Filter

The Particle Filter is a Monte Carlo-based method for state estimation in nonlinear systems with non-Gaussian noise. It represents the posterior distribution using a set of weighted samples.

### Moving Horizon Estimation (MHE)

MHE is an optimization-based approach that estimates the state trajectory over a finite horizon of past measurements, allowing for constraints and non-Gaussian noise.

## API Reference

For detailed API documentation, see:
- [LMS API](api/lms.md)
- [RLS API](api/rls.md)
- [Kalman Filter API](api/kalman.md)
- [Extended Kalman Filter API](api/ekf.md)
- [Unscented Kalman Filter API](api/ukf.md)
- [Particle Filter API](api/particle.md)
- [Moving Horizon Estimation API](api/mhe.md)

## License

This project is licensed under the MIT License - see the LICENSE file for details.
