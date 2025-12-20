#include <gtest/gtest.h>
#include "../include/Vector.h"

using namespace LinAlg;

TEST(VectorTests, TestSize) {
    LinAlg::Vector<int> test_vector(4);
    EXPECT_EQ(test_vector.get_size(), 4);
}

TEST(VectorTests, TestAccessAtIndex) {
    LinAlg::Vector<int> test_vector({3, 5, 2});
    EXPECT_EQ(test_vector[0], 3);
}