#include <gtest/gtest.h>
#include "../include/Matrix.h"

using namespace LinAlg;

class MatrixTests : public ::testing::Test {
 protected:
  LinAlg::Matrix<int> test_matrix;
  MatrixTests() : test_matrix({{3, 5, 2}, {2, 9, 13}, {0, 1, 9}}) {}
};

TEST_F(MatrixTests, TestRowsAndColumns) {
    EXPECT_EQ(test_matrix.get_rows(), 3);
    EXPECT_EQ(test_matrix.get_columns(), 3);
}

TEST_F(MatrixTests, TestAccessAtRowsAndColumns) {
    EXPECT_EQ(test_matrix(0, 1), 5);
}

TEST_F(MatrixTests, TestMatrixAssignment) {
    test_matrix(0, 1) = 3;
    EXPECT_EQ(test_matrix(0, 1), 3);
}

TEST_F(MatrixTests, TestOutOfRange) {
    EXPECT_THROW(test_matrix.at(3, 2), std::out_of_range);
}

TEST_F(MatrixTests, TestTranspose) {
    LinAlg::Matrix transpose_matrix = test_matrix.transpose();
    std::vector<std::vector<int>> expected = {{3, 2, 0}, {5, 9, 1}, {2, 13, 9}};
    EXPECT_EQ(transpose_matrix.get_values(), expected);
}