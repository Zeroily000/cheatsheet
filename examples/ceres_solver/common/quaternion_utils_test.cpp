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

TEST(QuaternionUtilsTest, TestQuaternionLeftUpdateJacobian) {
  Eigen::Quaterniond q0{Eigen::Quaterniond::UnitRandom()};

  Eigen::Matrix<double, 4, 3> const analytic{QuaternionLeftUpdateJacobian(q0)};

  Eigen::Matrix<double, 4, 3> numeric;
  Eigen::Vector3d const delta{Eigen::Vector3d::Random() * 1e-9};
  Eigen::Vector3d const delta_x{delta.x(), 0., 0.};
  Eigen::Quaterniond const dqxq0{Sophus::SO3d::exp(delta_x).unit_quaternion() * q0};
  numeric.col(0) = (dqxq0.coeffs() - q0.coeffs()) / delta.x();

  Eigen::Vector3d const delta_y{0., delta.y(), 0.};
  Eigen::Quaterniond const dqyq0{Sophus::SO3d::exp(delta_y).unit_quaternion() * q0};
  numeric.col(1) = (dqyq0.coeffs() - q0.coeffs()) / delta.y();

  Eigen::Vector3d const delta_z{0., 0., delta.z()};
  Eigen::Quaterniond const dqzq0{Sophus::SO3d::exp(delta_z).unit_quaternion() * q0};
  numeric.col(2) = (dqzq0.coeffs() - q0.coeffs()) / delta.z();

  EXPECT_TRUE(analytic.isApprox(numeric, 1e-6));
}

TEST(QuaternionUtilsTest, TestQuaternionRightUpdateJacobian) {
  Eigen::Quaterniond q0{Eigen::Quaterniond::UnitRandom()};

  Eigen::Matrix<double, 4, 3> const analytic{QuaternionRightUpdateJacobian(q0)};

  Eigen::Matrix<double, 4, 3> numeric;
  Eigen::Vector3d const delta{Eigen::Vector3d::Random() * 1e-9};
  Eigen::Vector3d const delta_x{delta.x(), 0., 0.};
  Eigen::Quaterniond const q0dqx{q0 * Sophus::SO3d::exp(delta_x).unit_quaternion()};
  numeric.col(0) = (q0dqx.coeffs() - q0.coeffs()) / delta.x();

  Eigen::Vector3d const delta_y{0., delta.y(), 0.};
  Eigen::Quaterniond const q0dqy{q0 * Sophus::SO3d::exp(delta_y).unit_quaternion()};
  numeric.col(1) = (q0dqy.coeffs() - q0.coeffs()) / delta.y();

  Eigen::Vector3d const delta_z{0., 0., delta.z()};
  Eigen::Quaterniond const q0dqz{q0 * Sophus::SO3d::exp(delta_z).unit_quaternion()};
  numeric.col(2) = (q0dqz.coeffs() - q0.coeffs()) / delta.z();

  EXPECT_TRUE(analytic.isApprox(numeric, 1e-6));
}
