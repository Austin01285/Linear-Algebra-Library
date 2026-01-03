#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "RigidBodyState.h"
#include "DynamicsEquations.h"
#include <cmath>
#include <iostream>

using namespace LinAlg;

void step(RigidBodyState& state, double dt) {
    RigidBodyState derivative = derivatives(state, 0.00);

    // Eular integration
    state.position = state.position + (derivative.position * dt);
    state.velocity_body = state.velocity_body + (derivative.velocity_body * dt);
    state.orientation = state.orientation + (derivative.orientation * dt);
    state.orientation = state.orientation.normalized();
    state.angular = state.angular + (derivative.angular * dt);
}

#endif