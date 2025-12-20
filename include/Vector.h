#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace LinAlg
{
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
        T& operator[](size_t index);
        const T& operator[](size_t index) const;
    };

    template<typename T>
    Vector<T>::Vector(size_t size) : _vals(size), _size(size) {}

    template<typename T>
    Vector<T>::Vector(const std::vector<T>& vals) : _vals(vals), _size(vals.size()) {}

    template<typename T>
    size_t Vector<T>::get_size() const {
        return _size;
    }

    template<typename T>
    T& Vector<T>::operator[](size_t index) {
        if (index >= _size) {
            throw std::out_of_range("Index " + std::to_string(index) + " is out of bounds of the vector.");
        }
        return _vals[index];
    }

    template<typename T>
    const T& Vector<T>::operator[](size_t index) const {
        if (index >= _size) {
            throw std::out_of_range("Index " + std::to_string(index) + " is out of bounds of the vector.");
        }
        return _vals[index];
    }

} // namespace LinAlg

#endif