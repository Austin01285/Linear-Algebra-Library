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

    class SquareMatrixException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit SquareMatrixException(const std::string& message)
            : std::runtime_error(message) {}
    };

    class SingularException : public std::runtime_error {
    public:
        // Use the base class constructor for easy message passing
        explicit SingularException(const std::string& message)
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

        // Addition/Subtraction
        Matrix<T> operator+(const Matrix<T>& second_matrix) const;
        Matrix<T> operator-(const Matrix<T>& second_matrix) const;

        // Multiplication
        Matrix<T> operator*(const Matrix<T>& second_matrix) const;
        Vector<T> operator*(const Vector<T>& vector) const;
        Matrix<T> operator*(T scalar) const;

        // Identity Matrix
        static Matrix<T> identity(size_t n) {
            Matrix<T> identity_matrix(n, n);
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (i == j) identity_matrix(i, j) = T(1);
                    else identity_matrix(i, j) = T(0);
                }
            }
            return identity_matrix;
        }
             
        // Determinant
        T determinant() const;

        // Inverse
        Matrix<double> inverse() const;

        // Ouput
        friend std::ostream& operator<<(std::ostream& out, const Matrix<T>& obj) {
            out << "[";
            for (size_t i = 0; i < obj._rows; i++) {
                out << "{";
                for (size_t j = 0; j < obj._cols; j++) {
                    out << obj._vals[i][j];
                    if (j < obj._cols - 1) {
                        out << ", ";
                    }
                }
                out << "}";
                if (i < obj._rows - 1) {
                    out << ", ";
                }
            }
            out << "]";
            return out;
        }

        // Trace
        T trace() const;

        // Diagonal Matrix
        static Matrix<T> diagonal(const Vector<T>& values) {
            Matrix<T> diagonal_matrix(values.get_size(), values.get_size());
            for (int i = 0; i < values.get_size(); i++) {
                for (int j = 0; j < values.get_size(); j++) {
                    if (i == j) diagonal_matrix(i, j) = values[i];
                    else diagonal_matrix(i, j) = 0;
                }
            }
            return diagonal_matrix;
        }

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

    // Faster but is unsafe for when the index is out of bounds
    template<typename T>
    T& Matrix<T>::operator()(size_t rows, size_t cols) {
        return _vals[rows][cols];
    }

    template<typename T>
    const T& Matrix<T>::operator()(size_t rows, size_t cols) const {
        return _vals[rows][cols];
    }

    // Slower but checks if the index is out of bounds
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

    // Finds the transpose of a matrix
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

    // Add two matrices
    template<typename T>
    Matrix<T> Matrix<T>::operator+(const Matrix<T>& second_matrix) const {
        if (_rows != second_matrix.get_rows()) {
            throw MultViolationException("The number of rows in the first matrix " + std::to_string(_rows) + " does not equal the number of rows " + std::to_string(second_matrix.get_rows()) +  " in the second matrix.");
        }
        if (_cols != second_matrix.get_columns()) {
            throw MultViolationException("The number of columns in the first matrix " + std::to_string(_cols) + " does not equal the number of columns " + std::to_string(second_matrix.get_columns()) +  " in the second matrix.");
        }
        Matrix<T> add_matrix(_rows, second_matrix.get_columns());
        for (int i = 0; i < add_matrix.get_rows(); ++i) {
            for (int j = 0; j < add_matrix.get_columns(); ++j) {
                add_matrix(i, j) = _vals[i][j] + second_matrix(i, j);
            }
        }
        return add_matrix;

    }

    // Subtract the first matrix by the second matrix
    template<typename T>
    Matrix<T> Matrix<T>::operator-(const Matrix<T>& second_matrix) const {
        if (_rows != second_matrix.get_rows()) {
            throw MultViolationException("The number of rows in the first matrix " + std::to_string(_rows) + " does not equal the number of rows " + std::to_string(second_matrix.get_rows()) +  " in the second matrix.");
        }
        if (_cols != second_matrix.get_columns()) {
            throw MultViolationException("The number of columns in the first matrix " + std::to_string(_cols) + " does not equal the number of columns " + std::to_string(second_matrix.get_columns()) +  " in the second matrix.");
        }
        Matrix<T> subtract_matrix(_rows, second_matrix.get_columns());
        for (int i = 0; i < subtract_matrix.get_rows(); ++i) {
            for (int j = 0; j < subtract_matrix.get_columns(); ++j) {
                subtract_matrix(i, j) = _vals[i][j] - second_matrix(i, j);
            }
        }
        return subtract_matrix;

    } 

    // Multiplies two matrices
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

    // Multiplies a matrix and a vector together and returns a vector
    template<typename T>
    Vector<T> Matrix<T>::operator*(const Vector<T>& vector) const {
        if (_cols != vector.get_size()) {
            throw MultViolationException("The number of columns in the matrix " + std::to_string(_cols) + " does not equal the number of values " + std::to_string(vector.get_size()) +  " in the vector.");
        }
        Vector<T> mult_vector(_rows);
        for (int i = 0; i < _rows; i++) {
            for (int j = 0; j < vector.get_size(); j++) {
                mult_vector[i] += _vals[i][j] * vector[j];
            }
        }
        return mult_vector;
    }

    template<typename T>
    Matrix<T> Matrix<T>::operator*(T scalar) const {
        Matrix<T> mult_matrix(_rows, _cols);
        for (int i = 0; i < _rows; i++) {
            for (int j = 0; j < _cols; j++) {
                mult_matrix(i, j) = _vals[i][j] * scalar;
            }
        }
        return mult_matrix;
    }

    // Finds the determinant of a square matrix
    template<typename T>
    T Matrix<T>::determinant() const {
        if (_rows != _cols) {
            throw SquareMatrixException("The matrix is not a square matrix (must have the same amount of rows and columns).");
        }
        if (_rows == 1) {
            return _vals[0][0];
        } else if (_rows == 2) {
            return _vals[0][0]*_vals[1][1] - _vals[0][1]*_vals[1][0];
        } else if (_rows == 3) {
            T first_det = _vals[0][0]*(_vals[1][1]*_vals[2][2] - _vals[1][2]*_vals[2][1]);
            T second_det = _vals[0][1]*(_vals[1][0]*_vals[2][2] - _vals[2][0]*_vals[1][2]);
            T third_det = _vals[0][2]*(_vals[1][0]*_vals[2][1] - _vals[2][0]*_vals[1][1]);
            return first_det - second_det + third_det;
        } else {
            throw std::out_of_range("Please manually simplify to a 3x3 Square Matrix or Smaller.");
        }
    }

    template<typename T>
    Matrix<double> Matrix<T>::inverse() const {
        Matrix<double> inverse_matrix(_rows, _cols);
        if (_rows != _cols) {
            throw SquareMatrixException("The matrix is not a square matrix (must have the same amount of rows and columns).");
        }
        double deter = static_cast<double>(this->determinant());
        double inv_det = 1.0/deter;
        if (abs(deter) < 1e-10 && _rows > 1) {
            throw SingularException("Matrix is singular or nearly singular.");
        }
        if (_rows == 1) {
            inverse_matrix(0, 0) = 1/_vals[0][0];
        } else if (_rows == 2) {
            inverse_matrix(0, 0) = (1.0/deter)*_vals[1][1];
            inverse_matrix(0, 1) = -(1.0/deter)*_vals[0][1];
            inverse_matrix(1, 0) = -(1.0/deter)*_vals[1][0];
            inverse_matrix(1, 1) = (1.0/deter)*_vals[0][0];
        } else if (_rows == 3) {
            // Row 0
            inverse_matrix(0, 0) = inv_det * (_vals[1][1]*_vals[2][2] - _vals[1][2]*_vals[2][1]);
            inverse_matrix(0, 1) = -inv_det * (_vals[0][1]*_vals[2][2] - _vals[0][2]*_vals[2][1]);
            inverse_matrix(0, 2) = inv_det * (_vals[0][1]*_vals[1][2] - _vals[0][2]*_vals[1][1]);
            
            // Row 1
            inverse_matrix(1, 0) = -inv_det * (_vals[1][0]*_vals[2][2] - _vals[1][2]*_vals[2][0]);
            inverse_matrix(1, 1) = inv_det * (_vals[0][0]*_vals[2][2] - _vals[0][2]*_vals[2][0]);
            inverse_matrix(1, 2) = -inv_det * (_vals[0][0]*_vals[1][2] - _vals[0][2]*_vals[1][0]);
            
            // Row 2
            inverse_matrix(2, 0) = inv_det * (_vals[1][0]*_vals[2][1] - _vals[1][1]*_vals[2][0]);
            inverse_matrix(2, 1) = -inv_det * (_vals[0][0]*_vals[2][1] - _vals[0][1]*_vals[2][0]);
            inverse_matrix(2, 2) = inv_det * (_vals[0][0]*_vals[1][1] - _vals[0][1]*_vals[1][0]);
        } else {
            throw std::out_of_range("Please manually simplify to a 3x3 Square Matrix or Smaller.");
        }
        return inverse_matrix;
    }

    template<typename T>
    T Matrix<T>::trace() const {
        if (_rows != _cols) {
            throw SquareMatrixException("The matrix is not a square matrix (must have the same amount of rows and columns).");
        }
        T sum = T(0);
        for (size_t i = 0; i < _rows; i++) {
            for (size_t j = 0; j < _cols; j++) {
                if (i == j) {
                    sum += _vals[i][j];
                }
            }
        }
        return sum;
    }



}


#endif