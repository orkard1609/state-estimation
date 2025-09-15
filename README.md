# State Estimation Library

This repository contains implementations of various state estimation algorithms, including Kalman filters, adaptive filters, particle filters, and Moving Horizon Estimation (MHE).

## Algorithms

- **Least Mean Squares (LMS)**: Adaptive filtering algorithm that updates filter coefficients to minimize the mean squared error
- **Recursive Least Squares (RLS)**: Adaptive algorithm with faster convergence than LMS
- **Kalman Filter**: Optimal estimator for linear systems with Gaussian noise
- **Extended Kalman Filter (EKF)**: Kalman filter extension for nonlinear systems using linearization
- **Unscented Kalman Filter (UKF)**: Kalman filter variant using the unscented transform for nonlinear systems
- **Particle Filter**: Non-parametric estimation technique using Monte Carlo sampling
- **Moving Horizon Estimation (MHE)**: Optimization-based estimation over a finite time horizon

## Directory Structure

```
state-estimation/
├── include/state_estimation/   # Header files
├── src/                        # Implementation files
├── examples/                   # Example applications
├── tests/                      # Unit tests
├── data/                       # Sample datasets
└── docs/                       # Documentation
```

## Building and Installation

This project uses CMake as its build system. To build the library and examples:

```bash
mkdir build && cd build
cmake ..
make
```

## Running Examples

After building, you can run the examples:

```bash
./lms_example
./kalman_linear
./ekf_nonlinear
# etc.
```

## Running Tests

```bash
ctest
```

## Documentation

See the docs/ directory for detailed documentation on each algorithm.
