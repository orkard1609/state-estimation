cmake_minimum_required(VERSION 3.14)
project(state_estimation VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add Eigen library (uncomment and modify if needed)
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

# Library source files
set(SOURCES
    src/LMS.cpp
    src/RLS.cpp
    src/Kalman.cpp
    src/ExtendedKalman.cpp
    src/UnscentedKalman.cpp
    src/ParticleFilter.cpp
    src/MovingHorizonEstimation.cpp
)

# Create state estimation library
add_library(state_estimation SHARED ${SOURCES})
target_include_directories(state_estimation PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Examples
add_executable(lms_example examples/lms_example.cpp)
target_link_libraries(lms_example state_estimation)

add_executable(rls_example examples/rls_example.cpp)
target_link_libraries(rls_example state_estimation)

add_executable(kalman_linear examples/kalman_linear.cpp)
target_link_libraries(kalman_linear state_estimation)

add_executable(ekf_nonlinear examples/ekf_nonlinear.cpp)
target_link_libraries(ekf_nonlinear state_estimation)

add_executable(ukf_nonlinear examples/ukf_nonlinear.cpp)
target_link_libraries(ukf_nonlinear state_estimation)

add_executable(particle_tracking examples/particle_tracking.cpp)
target_link_libraries(particle_tracking state_estimation)

add_executable(mhe_example examples/mhe_example.cpp)
target_link_libraries(mhe_example state_estimation)

# Enable testing
enable_testing()

# Add tests
add_executable(test_lms tests/test_lms.cpp)
target_link_libraries(test_lms state_estimation)
add_test(NAME LMSTest COMMAND test_lms)

add_executable(test_rls tests/test_rls.cpp)
target_link_libraries(test_rls state_estimation)
add_test(NAME RLSTest COMMAND test_rls)

add_executable(test_kalman tests/test_kalman.cpp)
target_link_libraries(test_kalman state_estimation)
add_test(NAME KalmanTest COMMAND test_kalman)

add_executable(test_ekf tests/test_ekf.cpp)
target_link_libraries(test_ekf state_estimation)
add_test(NAME EKFTest COMMAND test_ekf)

add_executable(test_ukf tests/test_ukf.cpp)
target_link_libraries(test_ukf state_estimation)
add_test(NAME UKFTest COMMAND test_ukf)

add_executable(test_particle tests/test_particle.cpp)
target_link_libraries(test_particle state_estimation)
add_test(NAME ParticleTest COMMAND test_particle)

add_executable(test_mhe tests/test_mhe.cpp)
target_link_libraries(test_mhe state_estimation)
add_test(NAME MHETest COMMAND test_mhe)

# Installation
install(TARGETS state_estimation
    EXPORT state_estimation-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
