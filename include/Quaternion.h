#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <stdexcept>
#include "Vector.h"
#include "Matrix.h"

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

    friend std::ostream& operator<<(std::ostream& os, const Quaternion<T>& q) {
        os << "Q(" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")";
        return os;
    }

    static Quaternion<T> identity() {
        return Quaternion(T(1), T(0), T(0), T(0));
    }

    // Named constructors that guarantee unit length (most common pattern)
    static Quaternion<T> fromAxisAngle(const Vector<T>& axis, T angle_rad) { // angle_rad must be radians
        if (axis.get_size() != 3) {
            throw std::invalid_argument("Invalid axis vector (size of vector should be 3)");
        }
        T axis_norm_sq = axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2];
        // Handle degenerate cases
        if (axis_norm_sq < T(1e-12))
        {
            // Invalid axis → return identity (common convention)
            return identity();
        }
        if (std::abs(angle_rad) < T(1e-10)) {
            return identity();
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

    static Quaternion<T> slerp(const Quaternion<T>& q0, const Quaternion<T>& q1, T t) {
        T cos_theta = q0.dot(q1); // Get the dot product of both quaternions
        T theta = std::acos(cos_theta); // Get the angle from that cos_theta
        T sin_theta = std::sin(theta); // Put that angle into a sinusodial
        T w = (sin((T(1) - t)*theta)/sin_theta)*q0.w() + ((sin(t*theta))/sin_theta)*q1.w();
        T x = (sin((T(1) - t)*theta)/sin_theta)*q0.x() + ((sin(t*theta))/sin_theta)*q1.x();
        T y = (sin((T(1) - t)*theta)/sin_theta)*q0.y() + ((sin(t*theta))/sin_theta)*q1.y();
        T z = (sin((T(1) - t)*theta)/sin_theta)*q0.z() + ((sin(t*theta))/sin_theta)*q1.z();
        return Quaternion(w, x, y, z);
    }

    // Getters
    T w() const { return w_; }
    T x() const { return x_; }
    T y() const { return y_; }
    T z() const { return z_; }
    T angle() const { return T(2) * std::acos(w_); } // Returns in radians
    Vector<T> axis() const { 
        T sin_half = std::sqrt(1-(w_*w_));
        if (sin_half < T(1e-6)) return Vector<T>({T(1), T(0), T(0)});
        return Vector<T>({x_/sin_half, y_/sin_half, z_/sin_half});
    }

    // Normalization
    Quaternion<T> normalized() const {
        T mag_sq = w_*w_ + x_*x_ + y_*y_ + z_*z_;
        if (mag_sq < T(1e-12)) {  // almost zero
            return identity();
        }
        T inv_mag = T(1) / std::sqrt(mag_sq);
        return Quaternion(w_*inv_mag, x_*inv_mag, y_*inv_mag, z_*inv_mag);
    }

    // Checks if the magnitude is at (or at least very close to) 1
    bool isUnit(T eps = T(1e-6)) const {
        T mag = std::sqrt(w_*w_ + x_*x_ + y_*y_ + z_*z_);
        return std::abs(mag - T(1)) < eps;
    }

    // Multiplying two Quaternions
    Quaternion<T> operator*(const Quaternion<T>& q) const;

    // Scalar multiplication
    Quaternion<T> operator*(T scalar) const;

    // Quaternion Addition
    Quaternion<T> operator+(const Quaternion<T>& q) const;

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

    // Dot product between two Quaternions
    T dot(const Quaternion<T>& quat) const;

    Vector<T> toEulerXYZ() {
        Quaternion<T> q = normalized();
        T sin_theta = T(2) * (q.w() * q.y() - q.z() * q.x());
        T bank, pitch, heading;
        if (std::abs(sin_theta) >= T(1.0 - 1e-10)) { // If the Euler angle is close to or at +/- 90
            bank = std::atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
            pitch = q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
            std::copysign(M_PI/2.0, sin_theta);
            heading = 0.00;
        } else {
            bank = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
            pitch = std::asin(sin_theta);
            heading = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        }
        return Vector<T>({bank, pitch, heading});
    }

    Quaternion<T> fromEulerXYZ(const Vector<T>& xyz) {
        if (xyz.get_size() != 3) {
            throw std::invalid_argument("Euler angles vector must have 3 components");
        }
        T cos_bank = std::cos(xyz[0]/T(2));
        T sin_bank = std::sin(xyz[0]/T(2));
        T cos_pitch = std::cos(xyz[1]/T(2));
        T sin_pitch = std::sin(xyz[1]/T(2));
        T cos_heading = std::cos(xyz[2]/T(2));
        T sin_heading = std::sin(xyz[2]/T(2));

        Quaternion<T> q;
        q.w_ = cos_bank * cos_pitch * cos_heading + sin_bank * sin_pitch * sin_heading;
        q.x_ = sin_bank * cos_pitch * cos_heading - cos_bank * sin_pitch * sin_heading;
        q.y_ = cos_bank * sin_pitch * cos_heading + sin_bank * cos_pitch * sin_heading;
        q.z_ = cos_bank * cos_pitch * sin_heading - sin_bank * sin_pitch * cos_heading;
        return q.normalized();
    }

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
    Quaternion<T> Quaternion<T>::operator*(T scalar) const {
        return Quaternion<T>(w_ * scalar, x_ * scalar, y_ * scalar, z_ * scalar);
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::operator+(const Quaternion<T>& q) const {
        return Quaternion<T>(w_ + q.w_, x_ + q.x_, y_ + q.y_, z_ + q.z_);
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

    template<typename T>
    T Quaternion<T>::dot(const Quaternion<T>& quat) const {
        return w_*quat.w() + x_*quat.x() + y_*quat.y() + z_*quat.z();
    }
}

#endif