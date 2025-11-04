#include "examples/ceres_solver/common/quaternion_utils.hpp"

#include <gtest/gtest.h>

TEST(QuaternionUtilsTest, TestQuaternionLeftMultiplicationMatrix) {
  Eigen::Quaterniond q0{Eigen::Quaterniond::UnitRandom()};
  Eigen::Quaterniond q1{Eigen::Quaterniond::UnitRandom()};

  EXPECT_TRUE((q0 * q1).coeffs().isApprox(QuaternionLeftMultiplicationMatrix(q0) * q1.coeffs()));
}

TEST(QuaternionUtilsTest, TestQuaternionRightMultiplicationMatrix) {
  Eigen::Quaterniond q0{Eigen::Quaterniond::UnitRandom()};
  Eigen::Quaterniond q1{Eigen::Quaterniond::UnitRandom()};

  EXPECT_TRUE((q0 * q1).coeffs().isApprox(QuaternionRightMultiplicationMatrix(q1) * q0.coeffs()));
}
