#pragma once

#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/quaternion_utils.h"

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionLeftMultiplicationMatrix(
    Eigen::QuaternionBase<Derived> const & quaternion) {
  // clang-format off
  /**
   * L(q) = [qw·I + [qv]x, qv]
   *        [       -qv^T, qw]
   */
  // clang-format on
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
  // clang-format off
  /**
   * R(q) = [qw·I - [qv]x, qv]
   *        [       -qv^T, qw]
   */
  // clang-format on
  using T = typename Derived::Scalar;
  Eigen::Matrix<T, 4, 4> R;
  R.template block<3, 3>(0, 0) =
      quaternion.w() * Eigen::Matrix<T, 3, 3>::Identity() - Sophus::SO3<T>::hat(quaternion.vec());
  R.template block<3, 1>(0, 3) = quaternion.vec();
  R.template block<1, 3>(3, 0) = -quaternion.vec().transpose();
  R(3, 3) = quaternion.w();
  return R;
}
