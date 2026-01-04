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

    RigidBodyState operator+(const RigidBodyState& state) const {
        RigidBodyState new_state;
        new_state.position = (*this).position + state.position;
        new_state.velocity_body = (*this).velocity_body + state.velocity_body;
        new_state.orientation = (*this).orientation + state.orientation;
        new_state.angular = (*this).angular + state.angular;
    }

    RigidBodyState operator*(double scalar) const {
        RigidBodyState new_state;
        new_state.position = (*this).position * scalar;
        new_state.velocity_body = (*this).velocity_body * scalar;
        new_state.orientation = (*this).orientation * scalar;
        new_state.angular = (*this).angular * scalar;
    }
};

#endif