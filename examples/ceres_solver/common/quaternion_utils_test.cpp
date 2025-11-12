#include "examples/ceres_solver/common/quaternion_utils.h"

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

TEST(QuaternionUtilsTest, TestQuaternionLeftUpdateJacobianInverse) {
  Eigen::Quaterniond q0{Eigen::Quaterniond::UnitRandom()};

  Eigen::Matrix<double, 3, 4> const analytic{QuaternionLeftUpdateJacobianInverse(q0)};

  Eigen::Matrix<double, 3, 4> numeric;
  Eigen::Vector4d const delta{Eigen::Vector4d::Random() * 1e-9};
  Eigen::Quaternion const dqx{0., delta.x(), 0., 0.};
  Eigen::Quaterniond const q0dqx{q0.coeffs() + dqx.coeffs()};
  numeric.col(0) = Sophus::SO3d{q0dqx * q0.inverse()}.log() / delta.x();

  Eigen::Quaternion const dqy{0., 0., delta.y(), 0.};
  Eigen::Quaterniond const q0dqy{q0.coeffs() + dqy.coeffs()};
  numeric.col(1) = Sophus::SO3d{q0dqy * q0.inverse()}.log() / delta.y();

  Eigen::Quaternion const dqz{0., 0., 0., delta.z()};
  Eigen::Quaterniond const q0dqz{q0.coeffs() + dqz.coeffs()};
  numeric.col(2) = Sophus::SO3d{q0dqz * q0.inverse()}.log() / delta.z();

  Eigen::Quaternion const dqw{delta.w(), 0., 0., 0.};
  Eigen::Quaterniond const q0dqw{q0.coeffs() + dqw.coeffs()};
  numeric.col(3) = Sophus::SO3d{q0dqw * q0.inverse()}.log() / delta.w();

  EXPECT_TRUE(analytic.isApprox(numeric, 1e-5));
  EXPECT_TRUE((analytic * QuaternionLeftUpdateJacobian(q0)).isApprox(Eigen::Matrix3d::Identity()));
}

TEST(QuaternionUtilsTest, TestQuaternionRightUpdateJacobianInverse) {
  Eigen::Quaterniond q0{Eigen::Quaterniond::UnitRandom()};

  Eigen::Matrix<double, 3, 4> const analytic{QuaternionRightUpdateJacobianInverse(q0)};

  Eigen::Matrix<double, 3, 4> numeric;
  Eigen::Vector4d const delta{Eigen::Vector4d::Random() * 1e-9};
  Eigen::Quaternion const dqx{0., delta.x(), 0., 0.};
  Eigen::Quaterniond const q0dqx{q0.coeffs() + dqx.coeffs()};
  numeric.col(0) = Sophus::SO3d{q0.inverse() * q0dqx}.log() / delta.x();

  Eigen::Quaternion const dqy{0., 0., delta.y(), 0.};
  Eigen::Quaterniond const q0dqy{q0.coeffs() + dqy.coeffs()};
  numeric.col(1) = Sophus::SO3d{q0.inverse() * q0dqy}.log() / delta.y();

  Eigen::Quaternion const dqz{0., 0., 0., delta.z()};
  Eigen::Quaterniond const q0dqz{q0.coeffs() + dqz.coeffs()};
  numeric.col(2) = Sophus::SO3d{q0.inverse() * q0dqz}.log() / delta.z();

  Eigen::Quaternion const dqw{delta.w(), 0., 0., 0.};
  Eigen::Quaterniond const q0dqw{q0.coeffs() + dqw.coeffs()};
  numeric.col(3) = Sophus::SO3d{q0.inverse() * q0dqw}.log() / delta.w();

  EXPECT_TRUE(analytic.isApprox(numeric, 1e-5));
  EXPECT_TRUE((analytic * QuaternionRightUpdateJacobian(q0)).isApprox(Eigen::Matrix3d::Identity()));
}
