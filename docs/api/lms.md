# LMS Filter API Reference

## Class: LMSFilter

### Description

The Least Mean Squares (LMS) filter is an adaptive filter that updates its filter coefficients to minimize the mean squared error between the output and a desired signal.

### Namespace

```cpp
namespace state_estimation
```

### Constructor

```cpp
LMSFilter(size_t filter_length, double step_size = 0.01);
```

**Parameters:**
- `filter_length`: The number of filter coefficients (taps)
- `step_size`: The step size (learning rate) for the LMS adaptation algorithm

**Exceptions:**
- `std::invalid_argument`: If filter_length is 0 or step_size is not positive

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
Resets the filter state, clearing the delay line and setting all coefficients to zero.

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

#### SetStepSize

```cpp
void setStepSize(double step_size);
```

**Description:**
Changes the step size (learning rate) for the adaptation algorithm.

**Parameters:**
- `step_size`: The new step size value

**Exceptions:**
- `std::invalid_argument`: If step_size is not positive

#### GetStepSize

```cpp
double getStepSize() const;
```

**Description:**
Returns the current step size.

**Returns:**
The current step size value

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
#include "state_estimation/LMS.hpp"
#include <vector>
#include <iostream>

int main() {
    // Create LMS filter with 16 taps and 0.05 step size
    state_estimation::LMSFilter lms(16, 0.05);
    
    // Initialize with all zeros (optional, reset() does this by default)
    std::vector<double> initial_coeffs(16, 0.0);
    lms.initialize(initial_coeffs);
    
    // Process a block of data
    std::vector<double> input = {/* input samples */};
    std::vector<double> desired = {/* desired output samples */};
    
    std::vector<double> output = lms.processBlock(input, desired);
    
    // Print final coefficients
    std::vector<double> coeffs = lms.getCoefficients();
    for (double c : coeffs) {
        std::cout << c << " ";
    }
    std::cout << std::endl;
    
    return 0;
}
```
