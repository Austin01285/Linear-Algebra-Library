cmake_minimum_required(VERSION 3.14)
project(MyTestProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Download Google Test automatically
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
include_directories("../include")
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/simulations/flight_dynamics)
# For Windows: prevents GTest from overriding your compiler's runtime settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE) 
FetchContent_MakeAvailable(googletest)

# 2. Enable testing
enable_testing()

# 3. Add your test executable
add_executable(test_vector tests/test_vector.cpp)
add_executable(test_matrix tests/test_matrix.cpp)
add_executable(test_quaternion tests/test_quaternion.cpp)
add_executable(6DoF_main simulations/flight_dynamics/main.cpp)

# 4. Link GTest libraries
target_link_libraries(test_vector GTest::gtest_main)
target_link_libraries(test_matrix GTest::gtest_main)
target_link_libraries(test_quaternion GTest::gtest_main)

# 5. Discover tests for the VS Code Test Explorer
include(GoogleTest)
gtest_discover_tests(test_vector)
gtest_discover_tests(test_matrix)
gtest_discover_tests(test_quaternion)