#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace LinAlg
{
    class DifferentDimensionException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit DifferentDimensionException(const std::string& message)
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
        template<typename U>
        auto operator+(const Vector<U>& second_vector) const;
        Vector operator-(const Vector& second_vector) const; // Subtracting Vectors
        template<typename U>
        auto operator-(const Vector<U>& second_vector) const;
        Vector operator*(T scalar) const; // Multiplying the vector by a scalar
        template<typename U>
        auto operator*(U scalar) const;

        // Dot Product
        Vector operator*(const Vector& second_vector) const;
        template<typename U>
        auto operator*(const Vector<U>& second_vector) const;
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

    // Vector addition if the vectors have different datatypes
    template<typename T>
    template<typename U>
    auto Vector<T>::operator+(const Vector<U>& second_vector) const {
        using ResultType = decltype(std::declval<T>() + std::declval<U>()); // Declares the type that happens when the vectors of different types are added
        if (second_vector.get_size() != _size) {
            throw DifferentDimensionException("Dimension of vector size " + std::to_string(_size) + " does not equal " + std::to_string(second_vector.get_size()));
        }
        
        Vector<ResultType> new_vector(_size);
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

    // Vector subtraction if the vectors have different datatypes
    template<typename T>
    template<typename U>
    auto Vector<T>::operator-(const Vector<U>& second_vector) const {
        using ResultType = decltype(std::declval<T>() - std::declval<U>()); // Declares the type that happens when the vectors of different types are added
        if (second_vector.get_size() != _size) {
            throw DifferentDimensionException("Dimension of vector size " + std::to_string(_size) + " does not equal " + std::to_string(second_vector.get_size()));
        }
        
        Vector<ResultType> new_vector(_size);
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

    template<typename T>
    template<typename U>
    auto Vector<T>::operator*(U scalar) const {
        using ResultType = decltype(std::declval<T>() * std::declval<U>()); // Declares the type that happens when the vectors of different types are added
        Vector<ResultType> new_vector(_size);
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

    // Vector subtraction if the vectors have different datatypes
    template<typename T>
    template<typename U>
    auto Vector<T>::operator*(const Vector<U>& second_vector) const {
        using ResultType = decltype(std::declval<T>() * std::declval<U>()); // Declares the type that happens when the vectors of different types are added
        if (second_vector.get_size() != _size) {
            throw DifferentDimensionException("Dimension of vector size " + std::to_string(_size) + " does not equal " + std::to_string(second_vector.get_size()));
        }
        
        Vector<ResultType> new_vector(_size);
        for (size_t i = 0; i < _size; i++) {
            new_vector[i] = _vals[i] * second_vector[i];
        }
        return new_vector;
    }
} // namespace LinAlg

#endif