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

TEST_F(MatrixTests, MultMatrixTest) {
    LinAlg::Matrix<int> second_matrix({{5, 10}, {2, 3}, {9, 1}});
    LinAlg::Matrix<int> mult_matrix = test_matrix*second_matrix;
    std::vector<std::vector<int>> expected = {{43, 47}, {145, 60}, {83, 12}};
    EXPECT_EQ(mult_matrix.get_values(), expected);
}


TEST_F(MatrixTests, MultVectorTest) {
    LinAlg::Vector<int> test_vector({1, 5, 7});
    LinAlg::Vector<int> mult_vector = test_matrix*test_vector;
    std::vector<int> expected = {42, 138, 68};
    EXPECT_EQ(mult_vector.get_values(), expected);
}

TEST_F(MatrixTests, MultViolationTest) {
    LinAlg::Matrix<int> second_matrix({{5, 20}, {2, 3}});
    EXPECT_THROW(test_matrix*second_matrix, MultViolationException);
    LinAlg::Vector<int> test_vector({2, 5});
    EXPECT_THROW(test_matrix*test_vector, MultViolationException);
}

TEST_F(MatrixTests, TestIdentityMatrix) {
    LinAlg::Matrix<int> identity_test = LinAlg::Matrix<int>::identity(2);
    std::vector<std::vector<int>> expected = {{1, 0}, {0, 1}};
    EXPECT_EQ(identity_test.get_values(), expected);
}