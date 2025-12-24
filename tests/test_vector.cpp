#include <gtest/gtest.h>
#include "../include/Vector.h"

using namespace LinAlg;

class VectorTests : public ::testing::Test {
 protected:
  LinAlg::Vector<int> test_vector;
  VectorTests() : test_vector({3, 5, 2}) {}
};

TEST_F(VectorTests, TestSize) {
    EXPECT_EQ(test_vector.get_size(), 3);
}

TEST_F(VectorTests, TestAccessAtIndex) {
    EXPECT_EQ(test_vector[0], 3);
}

TEST_F(VectorTests, TestVectorAssignment) {
    test_vector[0] = 4;
    EXPECT_EQ(test_vector[0], 4);
}

TEST_F(VectorTests, TestOutOfRange) {
    EXPECT_THROW(test_vector.at(4), std::out_of_range);
}

TEST_F(VectorTests, TestVectorAddition) {
    // Test for vectors of similar data types
    LinAlg::Vector<int> second_vector({9, 4, 2});
    LinAlg::Vector<int> sum_test_vector = test_vector + second_vector;
    std::vector<int> expected = {12, 9, 4};
    EXPECT_EQ(sum_test_vector.get_values(), expected);
}

TEST_F(VectorTests, TestVectorSubtraction) {
    // Test for vectors of similar data types
    LinAlg::Vector<int> second_vector({9, 4, 2});
    LinAlg::Vector<int> dif_test_vector = test_vector - second_vector;
    std::vector<int> expected = {-6, 1, 0};
    EXPECT_EQ(dif_test_vector.get_values(), expected);
}

TEST_F(VectorTests, TestVectorScalarMultiply) {
    // Test for vectors of similar data types
    LinAlg::Vector<int> scalar_test_vector = test_vector*3;
    std::vector<int> expected = {9, 15, 6};
    EXPECT_EQ(scalar_test_vector.get_values(), expected);
}

TEST_F(VectorTests, TestVectorDotProduct) {
    // Test for vectors of similar data types
    LinAlg::Vector<int> second_vector({9, 4, 2});
    LinAlg::Vector<int> dot_test_vector = test_vector * second_vector;
    std::vector<int> expected = {27, 20, 4};
    EXPECT_EQ(dot_test_vector.get_values(), expected);
}

TEST_F(VectorTests, VectorMagnitudeTest) {
    EXPECT_NEAR(test_vector.magnitude(), 6.16, 0.01);
}

TEST_F(VectorTests, VectorNormalizationTest) {
    LinAlg::Vector<double> normal_vector = test_vector.normalization();
    EXPECT_NEAR(normal_vector[0], 0.49, 0.01);   // Within 0.01
    EXPECT_NEAR(normal_vector[1], 0.81, 0.01);
    EXPECT_NEAR(normal_vector[2], 0.32, 0.01);
}

TEST_F(VectorTests, CrossProductTest) {
    LinAlg::Vector<int> second_vector({9, 4, 2});
    LinAlg::Vector<int> cross_test_vector = test_vector.cross(second_vector);
    std::vector<int> expected = {2, 12, -33};
    EXPECT_EQ(cross_test_vector.get_values(), expected);


}