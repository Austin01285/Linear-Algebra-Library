#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <stdexcept>
#include "../include/Vector.h"
#include "../include/Matrix.h"

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
        Vector<T> unit_axis = axis.normalization();
        T half_angle = angle_rad / T(2);
        return Quaternion (
        std::cos(half_angle), 
        unit_axis[0] * std::sin(half_angle), 
        unit_axis[1] * std::sin(half_angle),
        unit_axis[2] * std::sin(half_angle)
        );
    }

    // Turns rotation matrix to Quaternion using Shepperd's Method
    static Quaternion<T> from_rotation_matrix(const Matrix<T>& r) {
        if (r.get_rows() != 3 || r.get_columns() != 3) {
        throw std::invalid_argument("Rotation matrix must be 3×3");
        }
        T trace = r(0,0) + r(1,1) + r(2,2);
        T w, x, y, z;
        if (trace > T(0)) {
            // w is the largest component
            T s = std::sqrt(trace + T(1)) * T(2);  // s = 4*w
            w = T(0.25) * s;
            x = (r(2,1) - r(1,2)) / s;
            y = (r(0,2) - r(2,0)) / s;
            z = (r(1,0) - r(0,1)) / s;
        } 
        else if (r(0,0) > r(1,1) && r(0,0) > r(2,2)) {
            // x is the largest component
            T s = std::sqrt(T(1) + r(0,0) - r(1,1) - r(2,2)) * T(2);  // s = 4*x
            w = (r(2,1) - r(1,2)) / s;
            x = T(0.25) * s;
            y = (r(0,1) + r(1,0)) / s;
            z = (r(0,2) + r(2,0)) / s;
        } 
        else if (r(1,1) > r(2,2)) {
            // y is the largest component
            T s = std::sqrt(T(1) + r(1,1) - r(0,0) - r(2,2)) * T(2);  // s = 4*y
            w = (r(0,2) - r(2,0)) / s;
            x = (r(0,1) + r(1,0)) / s;
            y = T(0.25) * s;
            z = (r(1,2) + r(2,1)) / s;
        } 
        else {
            // z is the largest component
            T s = std::sqrt(T(1) + r(2,2) - r(0,0) - r(1,1)) * T(2);  // s = 4*z
            w = (r(1,0) - r(0,1)) / s;
            x = (r(0,2) + r(2,0)) / s;
            y = (r(1,2) + r(2,1)) / s;
            z = T(0.25) * s;
        }
        Quaternion<T> q(w, x, y, z);
        q.normalized();
        if (q.w() < T(0)) {
            q = Quaternion<T>(-q.w(), -q.x(), -q.y(), -q.z());
        }
        return q;
    }

    // Getters
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
    Quaternion<T> operator*(const Quaternion<T>& q) const;

    // Returns the Conjugate of the current Quaternion
    Quaternion<T> conjugate() const {
        return Quaternion(w(), -(x()), -(y()), -(z()));
    }

    // Returns the Inverse of the current Quaternion
    Quaternion<T> inverse() const {
        T mag_sq = w_*w_ + x_*x_ + y_*y_ + z_*z_;

        // if already (approximately) unit, then conjugate
        if (std::abs(mag_sq - T(1)) < T(1e-8))
        {
            return conjugate();
        }

        if (mag_sq < T(1e-12))  // Very small magnitude, close to 0
        {
            throw std::runtime_error("Cannot invert quaternion: magnitude near zero");
        }

        T inv_mag_sq = T(1) / mag_sq;

        return Quaternion(
            w_ * inv_mag_sq,
            -x_ * inv_mag_sq,
            -y_ * inv_mag_sq,
            -z_ * inv_mag_sq
        );
    }

    // Converts a quaternion to a 3×3 rotation matrix
    Matrix<T> to_rotation_matrix() const;

    // Rotate the inputted vector by the Quaternion
    Vector<T> rotate_vector(const Vector<T>& v) const;
};

    template<typename T>
    Quaternion<T> Quaternion<T>::operator*(const Quaternion<T>& q) const {
        return Quaternion(
            w() * q.w() - x() * q.x() - y() * q.y() - z() * q.z(),
            w() * q.x() + x() * q.w() + y() * q.z() - z() * q.y(),
            w() * q.y() - x() * q.z() + y() * q.w() + z() * q.x(),
            w() * q.z() + x() * q.y() - y() * q.x() + z() * q.w()
        );
    }

    template<typename T>
    Matrix<T> Quaternion<T>::to_rotation_matrix() const {
        // Auto-normalizes if not unit
        if (!isUnit(T(1e-6)))
        {
            return normalized().to_rotation_matrix();
        }
        T ww = w_ * w_;
        T xx = x_ * x_;
        T yy = y_ * y_;
        T zz = z_ * z_;

        T xy = x_ * y_;
        T xz = x_ * z_;
        T yz = y_ * z_;

        T wx = w_ * x_;
        T wy = w_ * y_;
        T wz = w_ * z_;

        Matrix<T> R(3, 3);

        // Row 0
        R(0, 0) = ww + xx - yy - zz;
        R(0, 1) = T(2) * (xy - wz);
        R(0, 2) = T(2) * (xz + wy);

        // Row 1
        R(1, 0) = T(2) * (xy + wz);
        R(1, 1) = ww - xx + yy - zz;
        R(1, 2) = T(2) * (yz - wx);

        // Row 2
        R(2, 0) = T(2) * (xz - wy);
        R(2, 1) = T(2) * (yz + wx);
        R(2, 2) = ww - xx - yy + zz;

        return R;
    }

    template<typename T>
    Vector<T> Quaternion<T>::rotate_vector(const Vector<T>& v) const {
        if (v.get_size() != 3) {
            throw std::invalid_argument("Invalid axis vector (size of vector should be 3)");
        }
        // Convert the vector into a real quaternion
        Quaternion<T> real_quat(T(0), v[0], v[1], v[2]);
        Quaternion<T> temp = (*this) * real_quat;
        Quaternion<T> result = temp * inverse();
        return Vector<T>({result.x(), result.y(), result.z()});
    }
}

#endif