#ifndef QUATERNION_H
#define QUATERNION_H

#include <vector>
#include "../include/Vector.h"

namespace LinAlg {
    template<typename T>
    class Quaternion {
        private:
        struct InternalQuaternion {
            T w, x, y, z;
        };

        public:
        InternalQuaternion reg_quat;
        InternalQuaternion norm_quat;
        Quaternion();
        Quaternion(T new_w, T new_x, T new_y, T new_z);

        // Normalization
        void normalize();

        // Getters
        T getRegW() { return reg_quat.w; }
        T getRegX() { return reg_quat.x; }
        T getRegY() { return reg_quat.y; }
        T getRegZ() { return reg_quat.z; }
        T getNormW() { return norm_quat.w; }
        T getNormX() { return norm_quat.x; }
        T getNormY() { return norm_quat.y; }
        T getNormZ() { return norm_quat.z; }
    };

    template<typename T>
    Quaternion<T>::Quaternion() {
        reg_quat.w = norm_quat.w = T(1);
        reg_quat.x = norm_quat.x = reg_quat.y = norm_quat.y = reg_quat.z = norm_quat.z = T(0);
    }

    template<typename T>
    Quaternion<T>::Quaternion(T new_w, T new_x, T new_y, T new_z) {
        reg_quat.w = new_w;
        reg_quat.x = new_x;
        reg_quat.y = new_y;
        reg_quat.z = new_z;
        if (std::sqrt(reg_quat.w*reg_quat.w + reg_quat.x*reg_quat.x + reg_quat.y*reg_quat.y + reg_quat.z*reg_quat.z) != T(1)) {
            (*this).normalize();
        }
    }

    template<typename T>
    void Quaternion<T>::normalize() {
        T mag = std::sqrt(reg_quat.w*reg_quat.w + reg_quat.x*reg_quat.x + reg_quat.y*reg_quat.y + reg_quat.z*reg_quat.z);
        if (mag < T(1e-6)) {
            norm_quat.w = T(1);
            norm_quat.x = norm_quat.y = norm_quat.z = T(0);
        } else {
            norm_quat.w = reg_quat.w / mag;
            norm_quat.x = reg_quat.x / mag;
            norm_quat.y = reg_quat.y / mag;
            norm_quat.z = reg_quat.z / mag;
        }
    }
}


#endif