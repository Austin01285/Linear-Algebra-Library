#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <iostream>

namespace LinAlg
{
    class DifferentDimensionException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit DifferentDimensionException(const std::string& message)
            : std::runtime_error(message) {}
    };

    class CrossProductDimensionException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit CrossProductDimensionException(const std::string& message)
            : std::runtime_error(message) {}
    };

    template<typename T>
    class Vector {
    private:
        std::vector<T> _vals;
        size_t _size;

    public:
        // Constructors
        Vector(size_t size);
        Vector(const std::vector<T>& vals);

        // Accessors
        size_t get_size() const;
        const std::vector<T>& get_values() const;
        T& operator[](size_t index);
        const T& operator[](size_t index) const;
        T& at(size_t index);
        const T& at(size_t index) const;

        // Arithmetic
        Vector operator+(const Vector& second_vector) const; // Adding Vectors
        Vector operator-(const Vector& second_vector) const; // Subtracting Vectors
        Vector operator*(T scalar) const; // Multiplying the vector by a scalar
        Vector operator/(T scalar) const; // Dividing the vector by a scalar

        // Dot Product
        Vector operator*(const Vector& second_vector) const;

        // Magnitude
        double magnitude() const;

        // Normalization
        Vector<double> normalization() const;

        // Cross Product
        Vector cross(const Vector& second_vector) const;

        // Output
        friend std::ostream& operator<<(std::ostream& out, const Vector<T>& obj) {
            out << "[";
            for (size_t i = 0; i < obj._size; i++) {
                out << obj._vals[i];
                if (i < obj._size - 1) {
                    out << ", ";
                }
            }
            out << "]";
            return out;
        }

        // Squared norm
        T normSquared() const;
    };

    // Constructor with a size_t parameter that makes a zero vector of row size parameter
    template<typename T>
    Vector<T>::Vector(size_t size) : _vals(size), _size(size) {}

    // Constructor with a vector data type parameter that is converted into a Linear Algebra vector
    template<typename T>
    Vector<T>::Vector(const std::vector<T>& vals) : _vals(vals), _size(vals.size()) {}

    // Returns the row size of the vector
    template<typename T>
    size_t Vector<T>::get_size() const {
        return _size;
    }

    // Returns the values in the vector
    template<typename T>
    const std::vector<T>& Vector<T>::get_values() const {
        return _vals;
    }

    // Faster but is unsafe for when the index is out of bounds
    template<typename T>
    T& Vector<T>::operator[](size_t index) {
        return _vals[index];
    }

    template<typename T>
    const T& Vector<T>::operator[](size_t index) const {
        return _vals[index];
    }

    // Slower but checks if the index is out of bounds
    template<typename T>
    T& Vector<T>::at(size_t index) {
        if (index >= _size) {
            throw std::out_of_range("Index " + std::to_string(index) + " is out of bounds of the vector.");
        }
        return _vals[index];
    }

    template<typename T>
    const T& Vector<T>::at(size_t index) const {
        if (index >= _size) {
            throw std::out_of_range("Index " + std::to_string(index) + " is out of bounds of the vector.");
        }
        return _vals[index];
    }

    // Vector addition if both vectors have the same datatype (either both int or both double)
    template<typename T>
    Vector<T> Vector<T>::operator+(const Vector<T>& second_vector) const {
        if (second_vector.get_size() != _size) {
            throw DifferentDimensionException("Dimension of vector size " + std::to_string(_size) + " does not equal " + std::to_string(second_vector.get_size()));
        }
        Vector<T> new_vector(_size);
        for (size_t i = 0; i < _size; i++) {
            new_vector[i] = _vals[i] + second_vector[i];
        }
        return new_vector;
    }

    // Vector subtraction if both vectors have the same datatype (either both int or both double)
    template<typename T>
    Vector<T> Vector<T>::operator-(const Vector<T>& second_vector) const {
        if (second_vector.get_size() != _size) {
            throw DifferentDimensionException("Dimension of vector size " + std::to_string(_size) + " does not equal " + std::to_string(second_vector.get_size()));
        }
        Vector<T> new_vector(_size);
        for (size_t i = 0; i < _size; i++) {
            new_vector[i] = _vals[i] - second_vector[i];
        }
        return new_vector;
    }

    template<typename T>
    Vector<T> Vector<T>::operator*(T scalar) const {
        Vector<T> new_vector(_size);
        for (size_t i = 0; i < _size; i++) {
            new_vector[i] = _vals[i] * scalar;
        }
        return new_vector;
    }

    // Vector subtraction if both vectors have the same datatype (either both int or both double)
    template<typename T>
    Vector<T> Vector<T>::operator*(const Vector<T>& second_vector) const {
        if (second_vector.get_size() != _size) {
            throw DifferentDimensionException("Dimension of vector size " + std::to_string(_size) + " does not equal " + std::to_string(second_vector.get_size()));
        }
        Vector<T> new_vector(_size);
        for (size_t i = 0; i < _size; i++) {
            new_vector[i] = _vals[i] * second_vector[i];
        }
        return new_vector;
    }

    // Division by scalar
    template<typename T>
    Vector<T> Vector<T>::operator/(T scalar) const {
        Vector<T> result(_size);
        for (size_t i = 0; i < _size; i++) {
            result[i] = _vals[i] / scalar;
        }
        return result;
    }

    // Calculates the magnitude of the vector
    template<typename T>
    double Vector<T>::magnitude() const { // The magnitude of a vector is always a double
        double total = 0.00;
        for (size_t i = 0; i < _size; i++) {
            total += _vals[i] * _vals[i];
        }
        if (total == 0.00) {
            return 0.00;
        }
        return std::sqrt(total);
    }

    // Normalize the vector by dividing each value in it by the magnitude of the vector
    template<typename T>
    Vector<double> Vector<T>::normalization() const {
        Vector<double> normalized_vector(_size);
        double magnitude = this->magnitude();
        for (size_t i = 0; i < _size; i++) {
            normalized_vector[i] = _vals[i] / magnitude;
        }
        return normalized_vector;
    }

    // Does a Cross Product of two 3D vectors
    template<typename T>
    Vector<T> Vector<T>::cross(const Vector<T>& second_vector) const {
        if (_size != 3) {
            throw CrossProductDimensionException("Dimension of first vector " + std::to_string(_size) + " needs to be 3 in order to be put into a cross product.");
        } else if (second_vector.get_size() != 3) {
            throw CrossProductDimensionException("Dimension of second vector " + std::to_string(second_vector.get_size()) + " needs to be 3 in order to be put into a cross product.");
        }
        Vector<T> cross_product_vector(3);
        cross_product_vector[0] = _vals[1]*second_vector[2] - _vals[2]*second_vector[1];
        cross_product_vector[1] = _vals[2]*second_vector[0] - _vals[0]*second_vector[2];
        cross_product_vector[2] = _vals[0]*second_vector[1] - _vals[1]*second_vector[0];
        return cross_product_vector;
    }

    template<typename T>
    T Vector<T>::normSquared() const {
        T sum = T(0);
        for (size_t i = 0; i < _size; i++) {
            sum += _vals[i] * _vals[i];
        }
        return sum;
    }

} // namespace LinAlg

#endif