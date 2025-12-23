#ifndef MATRIX_H
#define MATRIX_H

#include <vector>

namespace LinAlg 
{
    class MatrixDimensionException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit MatrixDimensionException(const std::string& message)
            : std::runtime_error(message) {}
    };

    template<typename T>
    class Matrix {
        private:
        std::vector<std::vector<T>> _vals;
        size_t _rows, _cols;

        public:
        // Constructors 
        Matrix(size_t rows, size_t cols);
        Matrix(const std::vector<std::vector<T>> vals);

        // Accessors
        size_t get_rows() const;
        size_t get_columns() const;
        const std::vector<std::vector<T>> get_values() const;
        T& operator()(size_t rows, size_t cols);

    };

    template<typename T>
    Matrix<T>::Matrix(size_t rows, size_t cols) {
        for (int i = 0; i < rows; i++) {
            _vals.push_back(std::vector<T>(cols, 0));
        }
        _rows = rows;
        _cols = cols;
    }

    template<typename T>
    Matrix<T>::Matrix(const std::vector<std::vector<T>> vals) {
        size_t rows = vals.size();
        size_t cols = vals[0].size();
        if (rows > 1) {
            for (int i = 1; i < rows; i++) {
                if (cols != vals[i].size()) {
                    throw MatrixDimensionException("Every row in the matrix doesn't have " + std::to_string(cols) + " elements.");
                }
            }
        }
        _vals = vals;
        _rows = rows;
        _cols = cols;
    }

    template<typename T>
    size_t Matrix<T>::get_rows() const {
        return _rows;
    }

    template<typename T>
    size_t Matrix<T>::get_columns() const {
        return _cols;
    }

    template<typename T>
    const std::vector<std::vector<T>> Matrix<T>::get_values() const {
        return _vals;
    }





}


#endif