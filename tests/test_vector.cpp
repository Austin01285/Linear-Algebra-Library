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

TEST_F(VectorTests, TestOutOfRange) {
    EXPECT_THROW(test_vector.at(4), std::out_of_range);
}

TEST_F(VectorTests, TestVectorAddition) {
    LinAlg::Vector<int> second_vector({9, 4, 2});
    LinAlg::Vector<int> sum_test_vector = test_vector + second_vector;
    std::vector<int> expected = {12, 9, 4};
    EXPECT_EQ(sum_test_vector.get_values(), expected);
}

TEST_F(VectorTests, TestVectorSubtraction) {
    LinAlg::Vector<int> second_vector({9, 4, 2});
    LinAlg::Vector<int> sum_test_vector = test_vector - second_vector;
    std::vector<int> expected = {-6, 1, 0};
    EXPECT_EQ(sum_test_vector.get_values(), expected);
}