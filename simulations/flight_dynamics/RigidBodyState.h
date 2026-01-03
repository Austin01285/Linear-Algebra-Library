#ifndef RIGIDBODYSTATE_H
#define RIGIDBODYSTATE_H

#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"

using namespace LinAlg;

struct RigidBodyState {
    Vector<double> position;  // Position in an inertial frame (x, y, z)
    Vector<double> velocity_body;  // Velocity in a body-fixed frame (u, v, w)
    Quaternion<double> orientation;  // Attitude as a quaternion
    Vector<double> angular;  // Angular rates in the body frame (p, q, r)

    // Total size = 12 (position 3 + velocity 3 + quaternion 4 + ang vel 3)
    RigidBodyState() : position(3), velocity_body(3), angular(3) {}
};

#endif