# 6-DoF Flight Dynamics Simulator

A simple but complete 6-DoF rigid-body simulation using the core math library.

### Features
- Full 6-DoF dynamics (position, velocity, attitude, angular rates)
- Quaternion-based attitude propagation (no gimbal lock)
- Basic forces: thrust, gravity, quadratic drag
- Euler integration (easy to upgrade to RK4)
- Outputs position, velocity, quaternion, Euler angles

## Coordinate Frame Conventions

This library uses the following standard aerospace conventions:

### Inertial Frame
- **North-East-Down (NED)**: 
  - X: North
  - Y: East
  - Z: Down (positive toward Earth center)
- Common in aircraft, missiles, UAVs, and flight dynamics

### Body Frame (Vehicle-Fixed)
- **Forward-Right-Down (FRD)**:
  - X: Forward (along fuselage, positive out the nose)
  - Y: Right (positive out the right wing)
  - Z: Down (positive toward belly)
- Standard for fixed-wing aircraft, missiles, and most UAVs

### Quaternion Convention
- Represents **body-to-inertial** rotation (q rotates vectors from body frame to inertial frame)
- Hamilton product (q₁ * q₂ means apply q₂ first, then q₁)
- Unit quaternions only (normalized after every operation)

### Euler Angles
- Sequence: **3-2-1 (ZYX)** → yaw (ψ), pitch (θ), roll (φ)
- Range: yaw [-π, π], pitch [-π/2, π/2], roll [-π, π]

These choices align with common aerospace standards (e.g., Stevens & Lewis, NASA flight dynamics).