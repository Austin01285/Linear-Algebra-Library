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