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

## Config File Reference (config.txt)

The 6-DoF flight dynamics simulator loads initial conditions and vehicle parameters from a simple key-value text file (`config.txt` by default, located in the project root).

### Format Rules
- Lines are in the format: `key = value`
- Lines starting with `#` are comments (ignored)
- Empty lines are ignored
- All angles are in **degrees** (internally converted to radians)
- All values are floating-point numbers (doubles)
- Units are **SI** (meters, kilograms, seconds, Newtons, kg·m²)

### Full Example config.txt (Current Defaults)

# 6-DoF Simulation Configuration

# Initial state (position in NED inertial frame, velocity in body frame)
initial_position_x     = 0.0          # North (m)
initial_position_y     = 0.0          # East (m)
initial_position_z     = 1000.0       # Down/Altitude above sea level (m)

initial_velocity_x     = 100.0        # Forward velocity (body X, m/s)
initial_velocity_y     = 0.0          # Side velocity (body Y, m/s)
initial_velocity_z     = 0.0          # Down velocity (body Z, m/s)

initial_pitch_deg      = 0.0          # Initial pitch angle (degrees, nose up positive)

initial_angvel_p       = 0.0          # Roll rate (rad/s)
initial_angvel_q       = 0.0          # Pitch rate (rad/s)
initial_angvel_r       = 0.0          # Yaw rate (rad/s)

# Simulation timing
time_step              = 0.01         # Fixed time step for integration (s)
max_time               = 30.0         # Maximum simulation duration (s)

# Vehicle physical parameters
mass                   = 250.0        # Total mass (kg)
reference_area         = 12.0         # Aerodynamic reference area (m², usually wing area)
aspect_ratio           = 7.0          # Wing aspect ratio (span² / area)
oswald_eff             = 0.85         # Oswald efficiency factor (induced drag correction)

# Inertia tensor (diagonal, kg·m²) - symmetric assumption
Ixx                    = 500.0        # Roll moment of inertia
Iyy                    = 1000.0       # Pitch moment of inertia
Izz                    = 1200.0       # Yaw moment of inertia
Ixz                    = 0.0          # Product of inertia (usually 0 for symmetric vehicles)

# Propulsion
thrust_magnitude       = 2000.0       # Constant thrust force (N)
thrust_dir_x           = 1.0          # Thrust direction (body X component)
thrust_dir_y           = 0.0          # Thrust direction (body Y component)
thrust_dir_z           = 0.0          # Thrust direction (body Z component)

# Aerodynamic coefficients (per radian unless noted)
CL_zero                = 0.0          # Zero-lift lift coefficient
CL_alpha               = 5.5          # Lift curve slope (per radian)
CD_zero                = 0.025        # Zero-lift (parasite) drag coefficient
CD_alpha               = 0.05         # Quadratic drag term coefficient
CY_beta                = -1.0         # Side force coefficient due to sideslip (negative = stable)
CM_zero                = -0.01        # Zero-lift pitching moment coefficient
CM_alpha               = -2.0         # Pitching moment due to angle of attack (negative = stable)
c_bar                  = 1.8          # Mean aerodynamic chord (reference length for pitch, m)
CL_beta                = -0.3         # Rolling moment due to sideslip (negative = dihedral effect)
CN_beta                = 0.25         # Yawing moment due to sideslip (positive = directional stability)

# Damping coefficients (per radian, scaled by q * S * span)
C_lp                   = -1.0         # Roll damping (negative = stable)
C_mq                   = -4.0         # Pitch damping (negative = stable)
C_nr                   = -0.5         # Yaw damping (negative = stable)


## Detailed Explanation of Each Variable
Initial State

initial_position_x/y/z: Starting position in NED inertial frame (m). Z is altitude above sea level (positive up).
initial_velocity_x/y/z: Initial velocity in body frame (FRD: Forward, Right, Down) (m/s).
initial_pitch_deg: Initial pitch angle in degrees (nose up positive).
initial_angvel_p/q/r: Initial body-frame angular rates (roll/pitch/yaw) in rad/s.

Simulation Control

time_step: Fixed time step for numerical integration (s). Smaller = more accurate, but slower.
max_time: Total simulation duration (s).

Vehicle Geometry & Mass

mass: Total vehicle mass (kg).
reference_area: Aerodynamic reference area (m²) — usually wing planform area.
aspect_ratio: Wing aspect ratio (span² / reference_area) — affects induced drag.
oswald_eff: Oswald efficiency factor — corrects induced drag (0.7–0.95 typical).

Inertia Tensor (Diagonal)

Ixx/Iyy/Izz: Moments of inertia about body X/Y/Z axes (kg·m²).
Ixz: Product of inertia (usually 0 for symmetric vehicles).

Propulsion

thrust_magnitude: Constant thrust force (N).
thrust_dir_x/y/z: Unit vector for thrust direction in body frame (usually [1,0,0]).

Aerodynamic Coefficients

CL_zero: Zero-lift lift coefficient (often 0 for symmetric airfoils).
CL_alpha: Lift curve slope (per radian) — how lift changes with angle of attack.
CD_zero: Zero-lift (parasite) drag coefficient.
CD_alpha: Quadratic drag term coefficient (induced drag approximation).
CY_beta: Side force coefficient due to sideslip (negative = lateral stability).
CM_zero: Zero-lift pitching moment coefficient.
CM_alpha: Pitching moment due to angle of attack (negative = static pitch stability).
c_bar: Mean aerodynamic chord (reference length for pitching moments, m).
CL_beta: Rolling moment due to sideslip (negative = dihedral effect).
CN_beta: Yawing moment due to sideslip (positive = directional stability).

Damping Coefficients

C_lp: Roll damping (negative = roll rate damping).
C_mq: Pitch damping (negative = pitch rate damping).
C_nr: Yaw damping (negative = yaw rate damping).

All coefficients are scaled internally by dynamic pressure q, reference area S, and appropriate lengths (c_bar or wing span).
Notes

Angles in config are degrees (internally converted to radians)
All other values are in SI units
Add more parameters later (e.g., control surface deflections, stall angles)

This config system makes the simulator flexible and easy to tune for different vehicles!

### Getting Started
```bash
# Build tests & demo
mkdir build && cd build
cmake ..
cmake --build build --config Debug --target 6DoF_main

# Run unit tests
cd ./build/Debug/
./test_matrix or ./test_quaternion or ./test_vector

# Run simple 6-DoF simulation
cd ./build/Debug/
./6DoF_main.exe config.txt
