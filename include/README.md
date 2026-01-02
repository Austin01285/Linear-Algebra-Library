# Core Library (include/)

This folder contains the header-only implementation of the reusable math classes:

- `Vector.h` — Dynamic & fixed-size vectors
- `Matrix.h` — General matrices with row-major storage
- `Quaternion.h` — Unit quaternions for 3D rotations

**Important**:
- These are **general-purpose math tools**
- All classes are templated on `T` (default `double`)
- Designed for performance-critical aerospace applications
- Coordinate conventions & usage are documented in the top-level README.md

For application demos (e.g., 6-DoF flight simulation), see the `simulations/` folder.