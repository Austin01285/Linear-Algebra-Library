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

TEST_F(QuaternionTests, TestVectorRotation) {
    Quaternion<double> q_z90 = Quaternion<double>::fromAxisAngle(Vector<double>({0.0, 0.0, 1.0}), M_PI / 2.0);
    Vector<double> v_x({1.0, 0.0, 0.0});           // along x-axis
    Vector<double> expected_y({0.0, 1.0, 0.0});    // should become along y-axis
    Vector<double> result = q_z90.rotate_vector(v_x);
    EXPECT_NEAR(result[0], expected_y[0], 1e-10);
    EXPECT_NEAR(result[1], expected_y[1], 1e-10);
    EXPECT_NEAR(result[2], expected_y[2], 1e-10);
}

Matrix<double> rotation_matrix_from_axis_angle(
    const Vector<double>& axis, double angle_rad) {
    Quaternion<double> q = Quaternion<double>::fromAxisAngle(axis, angle_rad);
    return q.to_rotation_matrix();
}

TEST_F(QuaternionTests, TestFromRotationMatrix) {
    // Testing 90 degree rotation around the x-axis
    Matrix<double> R = rotation_matrix_from_axis_angle(Vector<double>({1, 0, 0}), M_PI / 2.0);
    Quaternion<double> q = Quaternion<double>::from_rotation_matrix(R);
    EXPECT_NEAR(q.w(), 0.707106781187, 1e-8);   // ← 1e-8 is safe
    EXPECT_NEAR(q.x(), 0.707106781187, 1e-8);
    EXPECT_NEAR(q.y(), 0.0, 1e-10);
    EXPECT_NEAR(q.z(), 0.0, 1e-10);
    EXPECT_TRUE(q.isUnit(1e-8));

    // Testing 180 degree rotation around the y-axis
    R = rotation_matrix_from_axis_angle(Vector<double>({0, 1, 0}), M_PI);
    q = Quaternion<double>::from_rotation_matrix(R);
    EXPECT_NEAR(q.w(), 0.0, 1e-10);
    EXPECT_NEAR(q.x(), 0.0, 1e-10);
    EXPECT_NEAR(std::abs(q.y()), 1.0, 1e-10);  // y should be ±1
    EXPECT_NEAR(q.z(), 0.0, 1e-10);
    EXPECT_TRUE(q.isUnit(1e-8));
}

TEST_F(QuaternionTests, SlurpTest) {
    // q0 = identity, q1 = 90° around Z
    Quaternion<double> q0(1.0, 0.0, 0.0, 0.0);
    Quaternion<double> q1 = Quaternion<double>::fromAxisAngle(Vector<double>({0.0, 0.0, 1.0}), M_PI/2.0);

    // t = 0.5 → 45° rotation
    Quaternion<double> result = Quaternion<double>::slerp(q0, q1, 0.5);

    // Expected: w = cos(22.5°) ≈ 0.9239, z = sin(22.5°) ≈ 0.3827
    EXPECT_NEAR(result.w(), 0.923879532511, 1e-8);
    EXPECT_NEAR(result.x(), 0.0, 1e-10);
    EXPECT_NEAR(result.y(), 0.0, 1e-10);
    EXPECT_NEAR(result.z(), 0.382683432365, 1e-8);

    EXPECT_TRUE(result.isUnit(1e-8));
}

// Helper: Check if two vectors are approximately equal (component-wise)
bool vectorsApproxEqual(const Vector<double>& a, const Vector<double>& b, double tol = 1e-8) {
    if (a.get_size() != b.get_size()) return false;
    for (size_t i = 0; i < a.get_size(); ++i) {
        if (std::abs(a[i] - b[i]) > tol) return false;
    }
    return true;
}

TEST_F(QuaternionTests, TestAxisAngleGetter) {
    // Test 90 degrees around the Z-axis
    Quaternion<double> q_z90 = Quaternion<double>::fromAxisAngle(Vector<double>({0.0, 0.0, 1.0}), M_PI / 2.0);
    EXPECT_NEAR(q_z90.angle(), M_PI / 2.0, 1e-10);
    Vector<double> axis_z = q_z90.axis();
    EXPECT_TRUE(vectorsApproxEqual(axis_z, Vector<double>({0.0, 0.0, 1.0}), 1e-10));

    // Test 180 degrees around the X-axis
    Quaternion<double> q_x180 = Quaternion<double>::fromAxisAngle(Vector<double>({1.0, 0.0, 0.0}), M_PI);
    EXPECT_NEAR(q_x180.angle(), M_PI, 1e-10);
    Vector<double> axis_x = q_x180.axis();
    EXPECT_TRUE(vectorsApproxEqual(axis_x, Vector<double>({1.0, 0.0, 0.0}), 1e-10) 
    || vectorsApproxEqual(axis_x, Vector<double>({-1.0, 0.0, 0.0}), 1e-10));
}
