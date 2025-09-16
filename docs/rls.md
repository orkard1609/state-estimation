# RLS Filter API Reference

## Class: RLSFilter

### Description

The Recursive Least Squares (RLS) filter is an adaptive filter that recursively finds the filter coefficients that minimize a weighted linear least squares cost function. It is faster converging than the LMS algorithm but more computationally intensive.

### Namespace

```cpp
namespace state_estimation
```

### Constructor

```cpp
RLSFilter(size_t filter_length, double forgetting_factor = 0.99, double delta = 100.0);
```

**Parameters:**
- `filter_length`: The number of filter coefficients (taps)
- `forgetting_factor`: Factor controlling the memory of the algorithm (0.9 to 1.0)
- `delta`: Regularization parameter for the initial correlation matrix

**Exceptions:**
- `std::invalid_argument`: If filter_length is 0, forgetting_factor is not in (0,1] range, or delta is not positive

### Public Methods

#### Initialize

```cpp
void initialize(const std::vector<double>& initial_coeffs);
```

**Description:**
Initializes the filter with a set of coefficients.

**Parameters:**
- `initial_coeffs`: Vector containing the initial filter coefficients

**Exceptions:**
- `std::invalid_argument`: If the size of initial_coeffs doesn't match the filter length

#### Reset

```cpp
void reset();
```

**Description:**
Resets the filter state, clearing the delay line, setting all coefficients to zero, and reinitializing the correlation matrix.

#### Update

```cpp
double update(double input, double desired);
```

**Description:**
Processes one input sample, updates the filter coefficients, and returns the filtered output.

**Parameters:**
- `input`: The current input sample
- `desired`: The desired output for the current input

**Returns:**
The filtered output before coefficient adaptation

#### ProcessBlock

```cpp
std::vector<double> processBlock(
    const std::vector<double>& input, 
    const std::vector<double>& desired
);
```

**Description:**
Processes a block of samples and updates the filter coefficients after each sample.

**Parameters:**
- `input`: Vector of input samples
- `desired`: Vector of desired output samples

**Returns:**
Vector of filtered output samples

**Exceptions:**
- `std::invalid_argument`: If input and desired vectors have different sizes

#### GetCoefficients

```cpp
std::vector<double> getCoefficients() const;
```

**Description:**
Returns the current filter coefficients.

**Returns:**
Vector containing the current filter coefficients

#### SetForgettingFactor

```cpp
void setForgettingFactor(double forgetting_factor);
```

**Description:**
Changes the forgetting factor for the RLS algorithm.

**Parameters:**
- `forgetting_factor`: The new forgetting factor value (0.9 to 1.0)

**Exceptions:**
- `std::invalid_argument`: If forgetting_factor is not in (0,1] range

#### GetForgettingFactor

```cpp
double getForgettingFactor() const;
```

**Description:**
Returns the current forgetting factor.

**Returns:**
The current forgetting factor value

#### GetFilterLength

```cpp
size_t getFilterLength() const;
```

**Description:**
Returns the length of the filter (number of coefficients).

**Returns:**
The number of filter coefficients

### Example Usage

```cpp
#include "state_estimation/RLS.hpp"
#include <vector>
#include <iostream>

int main() {
    // Create RLS filter with 16 taps, forgetting factor of 0.98
    state_estimation::RLSFilter rls(16, 0.98, 100.0);
    
    // Initialize coefficients (optional, reset() initializes to zeros)
    std::vector<double> initial_coeffs(16, 0.0);
    rls.initialize(initial_coeffs);
    
    // Process data sample by sample
    for (int i = 0; i < 100; i++) {
        // In a real application, these would come from sensors/data
        double input = /* input sample */;
        double desired = /* desired output */;
        
        double output = rls.update(input, desired);
        
        std::cout << "Sample " << i << ": Output = " << output << std::endl;
    }
    
    // Print final coefficients
    std::vector<double> coeffs = rls.getCoefficients();
    std::cout << "Final filter coefficients: ";
    for (double c : coeffs) {
        std::cout << c << " ";
    }
    std::cout << std::endl;
    
    return 0;
}
```
