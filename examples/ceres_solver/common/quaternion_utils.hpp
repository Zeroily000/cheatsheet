#pragma once

#include <sophus/so3.hpp>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "examples/ceres_solver/common/quaternion_utils.h"

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionLeftMultiplicationMatrix(
    Eigen::QuaternionBase<Derived> const & quaternion) {
  using T = typename Derived::Scalar;
  Eigen::Matrix<T, 4, 4> L;
  L.template block<3, 3>(0, 0) =
      quaternion.w() * Eigen::Matrix<T, 3, 3>::Identity() + Sophus::SO3<T>::hat(quaternion.vec());
  L.template block<3, 1>(0, 3) = quaternion.vec();
  L.template block<1, 3>(3, 0) = -quaternion.vec().transpose();
  L(3, 3) = quaternion.w();
  return L;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionRightMultiplicationMatrix(
    Eigen::QuaternionBase<Derived> const & quaternion) {
  using T = typename Derived::Scalar;
  Eigen::Matrix<T, 4, 4> R;
  R.template block<3, 3>(0, 0) =
      quaternion.w() * Eigen::Matrix<T, 3, 3>::Identity() - Sophus::SO3<T>::hat(quaternion.vec());
  R.template block<3, 1>(0, 3) = quaternion.vec();
  R.template block<1, 3>(3, 0) = -quaternion.vec().transpose();
  R(3, 3) = quaternion.w();
  return R;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> QuaternionLeftUpdateJacobian(
    Eigen::QuaternionBase<Derived> const & quaternion) {
  using T = typename Derived::Scalar;
  CHECK_EQ(quaternion.norm(), static_cast<T>(1));
  return static_cast<T>(.5) *
         QuaternionRightMultiplicationMatrix(quaternion).template block<4, 3>(0, 0);
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> QuaternionRightUpdateJacobian(
    Eigen::QuaternionBase<Derived> const & quaternion) {
  using T = typename Derived::Scalar;
  CHECK_EQ(quaternion.norm(), static_cast<T>(1));
  return static_cast<T>(.5) *
         QuaternionLeftMultiplicationMatrix(quaternion).template block<4, 3>(0, 0);
}
