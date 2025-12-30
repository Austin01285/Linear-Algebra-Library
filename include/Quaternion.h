#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <stdexcept>
#include "../include/Vector.h"

namespace LinAlg {

template<typename T>
class Quaternion {
private:
    T w_, x_, y_, z_;

public:
    // Identity quaternion
    Quaternion() : w_(T(1)), x_(T(0)), y_(T(0)), z_(T(0)) {}

    // Raw components (used for deserialization, math)
    Quaternion(T w, T x, T y, T z) : w_(w), x_(x), y_(y), z_(z) {}

    // Named constructors that guarantee unit length (most common pattern)
    static Quaternion<T> fromAxisAngle(const Vector<T>& axis, T angle_rad) {
        if (axis.get_size() != 3) {
            throw std::invalid_argument("Invalid axis vector (size of vector should be 3)");
        }
        T axis_norm_sq = axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2];
        // Handle degenerate cases
        if (axis_norm_sq < T(1e-12))
        {
            // Invalid axis → return identity (common convention)
            return Quaternion(T(1), T(0), T(0), T(0));
        }
        if (std::abs(angle_rad) < T(1e-10)) {
            return Quaternion(T(1), T(0), T(0), T(0));
        }
        Vector<double> unit_axis = axis.normalization();
        T half_angle = angle_rad / T(2);
        return Quaternion (
        std::cos(half_angle), 
        unit_axis[0] * std::sin(half_angle), 
        unit_axis[1] * std::sin(half_angle),
        unit_axis[2] * std::sin(half_angle)
        );
    }

    // Getters – const, simple, return the current (possibly unnormalized) values
    T w() const { return w_; }
    T x() const { return x_; }
    T y() const { return y_; }
    T z() const { return z_; }

    // Normalization
    Quaternion<T> normalized() const {
        T mag_sq = w_*w_ + x_*x_ + y_*y_ + z_*z_;
        if (mag_sq < T(1e-12)) {  // almost zero
            return Quaternion(T(1), T(0), T(0), T(0)); // identity
        }
        T inv_mag = T(1) / std::sqrt(mag_sq);
        return Quaternion(w_*inv_mag, x_*inv_mag, y_*inv_mag, z_*inv_mag);
    }

    void normalize() {
        *this = normalized();
    }

    // Checks if the magnitude is at (or at least very close to) 1
    bool isUnit(T eps = T(1e-6)) const {
        T mag = std::sqrt(w_*w_ + x_*x_ + y_*y_ + z_*z_);
        return std::abs(mag - T(1)) < eps;
    }

    // Multiplying two Quaternions
    Quaternion<T> operator*(const Quaternion<T>& q) {
        T new_w = (*this).w()*q.w() - ((*this).x()*q.x() + (*this).y()*q.y() + (*this).z()*q.z());
        T new_x = (*this).w()*q.x() + q.w()*(*this).x() + ((*this).y()*q.z() - q.y()*(*this).z());
        T new_y = (*this).w()*q.y() + q.w()*(*this).y() + ((*this).z()*q.x() - q.z()*(*this).x());
        T new_z = (*this).w()*q.z() + q.w()*(*this).z() + ((*this).x()*q.y() - q.x()*(*this).y());
        return Quaternion(new_w, new_x, new_y, new_z);
    }

};

}

#endif