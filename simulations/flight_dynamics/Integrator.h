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

void step_euler(RigidBodyState& state, double dt, VehicleConfig& config) {
    RigidBodyState derivative = derivatives(state, 0.00, config);

    // Eular integration
    state.position = state.position + (derivative.position * dt);
    state.velocity_body = state.velocity_body + (derivative.velocity_body * dt);
    Quaternion<double> w_quat({0.00, state.angular[0], state.angular[1], state.angular[2]});
    Quaternion<double> quat_rate = state.orientation * w_quat * 0.5;
    state.orientation = state.orientation + quat_rate * dt;
    state.orientation = state.orientation.normalized();
    state.angular = state.angular + (derivative.angular * dt);
}

void step_rk4(RigidBodyState& state, double dt, double t, VehicleConfig& config) {
    // RK4 integration

    // k1 = f(t, y)
    RigidBodyState k1 = derivatives(state, t, config);

    // k2 = f(t + dt/2, y + k1*dt/2)
    RigidBodyState state_k2 = state + (k1 * (dt * 0.5));
    state_k2.orientation.normalized();  // Keep quaternion normalized
    RigidBodyState k2 = derivatives(state_k2, t + (dt * 0.5), config);

    // k3 = f(t + dt/2, y + k2*dt/2)
    RigidBodyState state_k3 = state + (k2 * (dt * 0.5));
    state_k3.orientation.normalized();
    RigidBodyState k3 = derivatives(state_k3, t + (dt * 0.5), config);

    // k4 = f(t + dt, y + k3*dt)
    RigidBodyState state_k4 = state + k3 * dt;
    state_k4.orientation.normalized();
    RigidBodyState k4 = derivatives(state_k4, t + dt, config);

    // y_next = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    RigidBodyState combined = k1 + k2*2 + k3*2 + k4;
    state = state + (combined * (dt * 1/6));

    state.orientation.normalized();
}


#endif