# Linear Algebra Library for Aerospace & Defense

A lightweight, header-based C++ linear algebra library with focus on performance-critical applications in flight dynamics, guidance, navigation & control (GNC), and attitude determination.

### Core Features
- Templated 'Vector<T>', 'Matrix<T>', 'Quaternion<T>'
- Vector operations (arithmetic (+/-/*), dot/cross Product, normalization)
- Matrix operations (arithmetic + vector multiplication, determinant, diagonal/identity)
- Robust quaternion operations (multiplication, rotation, axis-angle conversion)
- Euler angle conversions (XYZ sequnece)
- Basic 6-DoF flight dynamics demo (in `simulations/`)

### Why This Library?
- Designed with aerospace needs in mind: quaternions for singularity-free attitude, unit normalization, numerical stability
- Header-only for easy integration
- No external dependencies (except for tests: GoogleTest)
- Clear coordinate frame conventions (NED inertial, FRD body, body-to-inertial quaternions)

### Coordinate Conventions
- Inertial frame: **North-East-Down (NED)**
- Body frame: **Forward-Right-Down (FRD)**
- Quaternion: **body-to-inertial** rotation
- Euler angles: **3-2-1 (ZYX)** sequence — yaw, pitch, roll

### Getting Started
```bash
# Build tests & demo
mkdir build && cd build
cmake ..
cmake --build . --config Release

# Run unit tests
./tests/test_quaternion

# Run simple 6-DoF simulation
./simulations/flight_dynamics/flight_sim
