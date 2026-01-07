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

struct VehicleConfig {
    double mass;
    Matrix<double> inertia_tensor;
    double reference_area;
    double CL_alpha;  // CLα
    double CD_alpha;  // CDα
    double CL_zero;  // CL₀
    double CD_zero;  // CD₀
    double CY_beta;  // CYβ
    double CM_alpha;  // Cmα
    double CM_zero;  // Cm0
    double c_bar;
    double CL_beta;  // Clβ
    double CN_beta;  // Cnβ
    double C_lp;
    double C_nr;
    double C_mq;
    double initial_alpha;
    double aspect_ratio;
    double oswald_eff;
    double thrust_magnitude;
    Vector<double> thrust_direction;

    VehicleConfig() : mass(250.00), inertia_tensor(3, 3), reference_area(12.0), 
    CL_alpha(5.5), CD_alpha(0.05), CL_zero(0.0), CD_zero(0.025), CY_beta(-1.0), 
    CM_alpha(-2.0), CM_zero(-0.01), c_bar(1.8), CL_beta(-0.3), CN_beta(0.25),
    C_lp(-1.0), C_nr(-0.5), C_mq(-4.0), aspect_ratio(7.0), oswald_eff(0.85), 
    thrust_magnitude(2000.0), thrust_direction({1.0, 0.0, 0.0}) {}
};

std::vector<double> ISA_Model(double altitude) {
    double temperature = 0;
    double pressure = 0;
    double density = 0;
    if (altitude < 1e-6) {  // Altitude is near sea level
        temperature = 288.15;  // In Kelvin, also 15 in Celcius
        pressure = 101325;  // In Pa
        density = 1.225;  // In kg/m^3
        return {density, temperature, pressure, 
            std::sqrt(1.4 * 287.05 * temperature)}; // Last return value is speed of sound
    } else if (altitude < 11000) {
        temperature = 288.15 + (-0.0065 * altitude);
        pressure = 101325 * std::pow(
            temperature/288.15, -9.80665/(287.05*-0.0065));  // Hydrostatic equation
        density = pressure/(287.05*temperature);  // Ideal Gas Law
        return {density, temperature, pressure, 
            std::sqrt(1.4 * 287.05 * temperature)};
    } else {
        temperature = 216.65;
        pressure = 22632 * std::exp(
            -9.80665 * (altitude - 11000) / (287.05 * 216.65)); // 22632 is the pressure at 11 km
        density = pressure/(287.05*temperature);
        return {density, temperature, pressure, 
            std::sqrt(1.4 * 287.05 * temperature)};
    }
}

ForcesMoments compute_forces_moments(const RigidBodyState& state, double t, const VehicleConfig& config) {
    ForcesMoments fm;

    // Applying thrust to body frame
    fm.force_body = config.thrust_direction * config.thrust_magnitude;

    // Applying gravity to body frame = R^T * [0, 0, -g*mass] (inertial frame)
    Vector<double> gravity_inertial({0.00, 0.00, -9.81*config.mass});
    Matrix<double> R = state.orientation.to_rotation_matrix();
    Vector<double> gravity_body = R.transpose()*gravity_inertial;
    fm.force_body = fm.force_body + gravity_body;

    // Get Dynamic Pressure from applying ISA
    std::vector<double> isa_values = ISA_Model(state.position[2]);

    // Get angle of attack (alpha) and sideslip (beta)
    double speed_sq = state.velocity_body.normSquared();
    double alpha;
    double beta;
    double q;
    if (speed_sq > 1e-6) {
        alpha = std::atan2(state.velocity_body[2], state.velocity_body[0]); // alpha = angle of attack
        beta = std::asin(state.velocity_body[1]/state.velocity_body.magnitude());
        q = 0.5 * isa_values[0] * speed_sq;
    } else {
        alpha = 0.0;
        beta = 0.0;
        q = 0.0;
    }

    // Find Lift and Drag Magnitudes
    double lift_coeff = config.CL_zero + config.CL_alpha * alpha;
    double drag_coeff = config.CD_zero + (lift_coeff * lift_coeff) / (
        M_PI * config.aspect_ratio * config.oswald_eff); 
    double side_force_coeff = config.CY_beta * beta;
    double lift_mag = lift_coeff * q * config.reference_area;
    double drag_mag = drag_coeff * q * config.reference_area;
    double side_force_mag = side_force_coeff * q * config.reference_area;  // Find Side Force

    // Find Lift and Drag Vectors
    Vector<double> drag_vector(3);
    Vector<double> lift_vector = Vector<double>({-std::sin(alpha), 0.0, std::cos(alpha)});
    if (speed_sq > 1e-6) {
        drag_vector = state.velocity_body.normalization() * -drag_mag;
    }
    Vector<double> new_lift_vector(3);
    if (beta > 10) {
        new_lift_vector = lift_vector;
    } else {
        new_lift_vector[0] = lift_vector[0] * std::cos(beta) - lift_vector[1] * std::sin(beta);
        new_lift_vector[1] = lift_vector[0] * std::sin(beta) + lift_vector[1] * std::cos(beta);
        new_lift_vector[2] = lift_vector[2];
    }
    lift_vector = new_lift_vector * lift_mag;
    fm.force_body = fm.force_body + drag_vector + lift_vector;
    fm.force_body[1] = fm.force_body[1] + side_force_mag;

    // Aerodynamic Moments
    double pitching_moment_coeff = config.CM_zero + config.CM_alpha * alpha;
    double rolling_moment_coeff = config.CL_beta * beta;
    double yawing_moment_coeff = config.CN_beta * beta;
    double wing_span = std::sqrt(config.aspect_ratio * config.reference_area);
    fm.moment_body[1] += pitching_moment_coeff * q * config.reference_area * config.c_bar;
    fm.moment_body[0] += rolling_moment_coeff * q * config.reference_area * wing_span;
    fm.moment_body[2] += yawing_moment_coeff * q * config.reference_area * wing_span;

    // Damping Moments
    fm.moment_body[0] += config.C_lp * state.angular[0] * q * config.reference_area * wing_span;
    fm.moment_body[1] += config.C_mq * state.angular[1] * q * config.reference_area * wing_span;
    fm.moment_body[2] += config.C_nr * state.angular[2] * q * config.reference_area * wing_span;

    return fm;
}

RigidBodyState derivatives(const RigidBodyState& state, double t, const VehicleConfig& config) {
    // Translational kinematics (inertial frame)
    RigidBodyState d;
    ForcesMoments fm = compute_forces_moments(state, t, config);
    Matrix<double> R = state.orientation.to_rotation_matrix();
    d.position = R * state.velocity_body;  // Position derivative = rotation matrix x velocity body

    // Translational dynamics (body frame)
    double mass = config.mass;
    d.velocity_body = (fm.force_body / mass) - (d.angular.cross(state.velocity_body));

    // Rotational kinematics (quaternion)
    Quaternion<double> omega_quat(0.0, state.angular[0], state.angular[1], state.angular[2]);
    d.orientation = (state.orientation * omega_quat) * 0.5;

    // Rotational dynamics
    Matrix<double> inertia = config.inertia_tensor;  // kg·m²
    Vector<double> inertia_omega = inertia * state.angular;
    Vector<double> inertia_cross_angular = inertia_omega.cross(state.angular);
    d.angular = inertia.inverse() * (fm.moment_body - inertia_cross_angular);

    return d;
}

#endif