#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <gtest/gtest.h>
#include "../include/Quaternion.h"
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>

using namespace LinAlg;

class QuaternionTests : public ::testing::Test {
 protected:
  LinAlg::Quaternion<double> test_quat;
  QuaternionTests() : test_quat(1.25, 0.00, 0.75, 0.00) {}
};

TEST_F(QuaternionTests, TestRegularScalarGetters) {
    EXPECT_EQ(test_quat.w(), 1.25);
    EXPECT_EQ(test_quat.x(), 0.00);
    EXPECT_EQ(test_quat.y(), 0.75);
    EXPECT_EQ(test_quat.z(), 0.00);
}

TEST_F(QuaternionTests, TestNormalizationScalarGetters) {
    EXPECT_EQ(test_quat.isUnit(), false);
    test_quat.normalize();
    EXPECT_NEAR(test_quat.w(), 0.85, 0.01);
    EXPECT_NEAR(test_quat.x(), 0.00, 0.01);
    EXPECT_NEAR(test_quat.y(), 0.51, 0.01);
    EXPECT_NEAR(test_quat.z(), 0.00, 0.01);
}

TEST_F(QuaternionTests, TestAxisAngleConstructor) {
    Vector<double> axis({0, 0, 1});  // Z-axis
    auto q = Quaternion<double>::fromAxisAngle(axis, M_PI / 2.0);  // 90 degrees
    EXPECT_NEAR(q.w(), 0.70710678118, 1e-10);   // cos(45°)
    EXPECT_NEAR(q.x(), 0.0, 1e-10);
    EXPECT_NEAR(q.y(), 0.0, 1e-10);
    EXPECT_NEAR(q.z(), 0.70710678118, 1e-10);   // sin(45°)
    Vector<double> invalid_axis({2, 0, 2, 1});
    EXPECT_THROW(Quaternion<double>::fromAxisAngle(invalid_axis, M_PI / 2.0), std::invalid_argument);
}

TEST_F(QuaternionTests, TestQuaternionMult) {
    Quaternion<double> qz90 = Quaternion<double>::fromAxisAngle(Vector<double>({0,0,1}), M_PI/2);
    Quaternion<double> qz180 = qz90 * qz90;
    EXPECT_NEAR(qz180.w(), 0.00, 1e-10);
    EXPECT_NEAR(qz180.x(), 0.00, 1e-10);
    EXPECT_NEAR(qz180.y(), 0.00, 1e-10);
    EXPECT_NEAR(qz180.z(), 1.00, 1e-10);
}

TEST_F(QuaternionTests, TestConjugate) {
    Quaternion<double> conjugate_test = test_quat.conjugate();
    EXPECT_EQ(conjugate_test.w(), 1.25);
    EXPECT_EQ(conjugate_test.x(), 0.00);
    EXPECT_EQ(conjugate_test.y(), -0.75);
    EXPECT_EQ(conjugate_test.z(), 0.00);
}

TEST_F(QuaternionTests, TestInverse) {
    Quaternion<double> inverse_test = test_quat.inverse();

    // test quaternion * inverse of test should be close to identity (q*q^-1 = I)
    Quaternion<double> product = test_quat * inverse_test;

    EXPECT_NEAR(product.w(), 1.0, 1e-10);
    EXPECT_NEAR(product.x(), 0.0, 1e-10);
    EXPECT_NEAR(product.y(), 0.0, 1e-10);
    EXPECT_NEAR(product.z(), 0.0, 1e-10);
}