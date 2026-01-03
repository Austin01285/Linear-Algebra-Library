#ifndef DYNAMICSEQUATIONS_H
#define DYNAMICSEQUATIONS_H

#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "RigidBodyState.h"
#include <cmath>
#include <iostream>

using namespace LinAlg;

struct ForcesMoments {
    Vector<double> force_body;     // thrust + aero + gravity in body frame
    Vector<double> moment_body;    // control + aero moments

    ForcesMoments() : force_body(3), moment_body(3) {}

    ForcesMoments(const Vector<double>& forces, const Vector<double>& moments) :
    force_body(forces), moment_body(moments) {}
};

ForcesMoments compute_forces_moments(const RigidBodyState& state, double t) {
    ForcesMoments fm;

    // Example: Constant north thrust (along the x-axis)
    fm.force_body = Vector<double>({1000.00, 0.0, 0.0});

    // Gravity in body frame = R^T * [0, 0, -g*mass] (inertial frame)
    Vector<double> gravity_inertial({0.00, 0.00, -9.81*100}); // mass = 100 kg
    Matrix<double> R = state.orientation.to_rotation_matrix();
    Vector<double> gravity_body = R.transpose()*gravity_inertial;
    fm.force_body = fm.force_body + gravity_body;

    // Simple quadratic drag (proportional to v²)
    double drag_coeff = 0.1; // This represents: ½ × ρ × C_d × A
    double speed_sq = state.velocity_body.normSquared();
    if (speed_sq > 1e-6) {
        Vector<double> drag = state.velocity_body.normalization() * (-drag_coeff * speed_sq);
        // drag = -k × |v|² × (v/|v|)
        //      = -k × |v|² × v̂  (where v̂ is the unit vector)
        //      = -k × |v| × v  (final simplified form)
        fm.force_body = fm.force_body + drag;
    }

    // Pitch damping moment
    fm.moment_body = Vector<double>({0.0, -0.1 * state.angular[1], 0.0});
    //                                    ^^^^               ^^^^^
    //                                    |                  pitch rate (q)
    //                                    damping coefficient (lumps C_mq, q_bar, S, c_bar)
    return fm;
}

RigidBodyState derivatives(const RigidBodyState& state, double t) {
    // Translational kinematics (inertial frame)
    RigidBodyState d;
    ForcesMoments fm = compute_forces_moments(state, t);
    Matrix<double> R = state.orientation.to_rotation_matrix();
    d.position = R * state.velocity_body;  // Position derivative = rotation matrix x velocity body

    // Translational dynamics (body frame)
    double mass = 100; // mass = 100 kg
    d.velocity_body = (fm.force_body / mass) - (d.angular.cross(state.velocity_body));

    // Rotational kinematics (quaternion)
    Quaternion<double> omega_quat(0.0, state.angular[0], state.angular[1], state.angular[2]);
    d.orientation = (state.orientation * omega_quat) * 0.5;

    // Rotational dynamics
    Matrix<double> inertia = Matrix<double>::identity(3) * 10.0;  // kg·m²
    Vector<double> inertia_omega = inertia * state.angular;
    Vector<double> inertia_cross_angular = inertia_omega.cross(state.angular);
    d.angular = inertia.inverse() * (fm.moment_body - inertia_cross_angular);

    return d;
}

#endif