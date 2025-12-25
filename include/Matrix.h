#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include "../include/Vector.h"

namespace LinAlg 
{
    class MatrixDimensionException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit MatrixDimensionException(const std::string& message)
            : std::runtime_error(message) {}
    };

    class MultViolationException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit MultViolationException(const std::string& message)
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
        const T& operator()(size_t rows, size_t cols) const;
        T& at(size_t rows, size_t cols);
        const T& at(size_t rows, size_t cols) const;

        // Transpose
        Matrix<T> transpose() const;

        // Multiplication
        Matrix<T> operator*(const Matrix<T>& second_matrix) const;
        Vector<T> operator*(const Vector<T>& vector) const;

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

    template<typename T>
    T& Matrix<T>::operator()(size_t rows, size_t cols) {
        return _vals[rows][cols];
    }

    template<typename T>
    const T& Matrix<T>::operator()(size_t rows, size_t cols) const {
        return _vals[rows][cols];
    }

    template<typename T>
    T& Matrix<T>::at(size_t rows, size_t cols) {
        if (rows >= _rows) {
            throw std::out_of_range("The " + std::to_string(rows) + " is out of bounds of the Matrix.");
        } else if (cols >= _cols) {
            throw std::out_of_range("The " + std::to_string(cols) + " is out of bounds of the Matrix.");
        }
        return _vals[rows][cols];
    }

    template<typename T>
    const T& Matrix<T>::at(size_t rows, size_t cols) const {
        if (rows >= _rows) {
            throw std::out_of_range("The " + std::to_string(rows) + " is out of bounds of the Matrix.");
        } else if (cols >= _cols) {
            throw std::out_of_range("The " + std::to_string(cols) + " is out of bounds of the Matrix.");
        }
        return _vals[rows][cols];
    }

    template<typename T>
    Matrix<T> Matrix<T>::transpose() const {
        Matrix<T> trans_matrix(_cols, _rows);
        for (int i = 0; i < trans_matrix.get_rows(); i++) {
            for (int j = 0; j < trans_matrix.get_columns(); j++) {
                if (i == j) {
                    trans_matrix(i, j) = _vals[i][j];
                } else {
                    trans_matrix(i, j) = _vals[j][i];
                }
            }
        }
        return trans_matrix;
    }

    // Does the cross product of two matrices
    template<typename T>
    Matrix<T> Matrix<T>::operator*(const Matrix<T>& second_matrix) const {
        if (_cols != second_matrix.get_rows()) {
            throw MultViolationException("The number of columns in the first matrix " + std::to_string(_cols) + " does not equal the number of rows " + std::to_string(second_matrix.get_rows()) +  " in the second matrix.");
        }
        Matrix<T> mult_matrix(_rows, second_matrix.get_columns());
        for (int i = 0; i < mult_matrix.get_rows(); ++i) {
            for (int j = 0; j < mult_matrix.get_columns(); ++j) {
                for (int k = 0; k < _cols; ++k) {
                    mult_matrix(i, j) += _vals[i][k] * second_matrix(k, j);
                }
            }
        }
        return mult_matrix;
    }


}


#endif