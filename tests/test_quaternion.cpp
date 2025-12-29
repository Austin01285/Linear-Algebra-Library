#include <gtest/gtest.h>
#include "../include/Quaternion.h"
#include <string>
#include <sstream>
#include <iostream>

using namespace LinAlg;

class QuaternionTests : public ::testing::Test {
 protected:
  LinAlg::Quaternion<double> test_quat1;
  LinAlg::Quaternion<double> test_quat2;
  QuaternionTests() : test_quat1(), test_quat2(1.25, 0.00, 0.75, 0.00) {}
};

TEST_F(QuaternionTests, TestRegularScalarGetters) {
    EXPECT_EQ(test_quat1.getRegW(), 1.00);
    EXPECT_EQ(test_quat1.getRegX(), 0.00);
    EXPECT_EQ(test_quat1.getRegY(), 0.00);
    EXPECT_EQ(test_quat1.getRegZ(), 0.00);
}

TEST_F(QuaternionTests, TestNormalizationScalarGetters) {
    EXPECT_NEAR(test_quat2.getNormW(), 0.85, 0.01);
    EXPECT_NEAR(test_quat2.getNormX(), 0.00, 0.01);
    EXPECT_NEAR(test_quat2.getNormY(), 0.51, 0.01);
    EXPECT_NEAR(test_quat2.getNormZ(), 0.00, 0.01);
}