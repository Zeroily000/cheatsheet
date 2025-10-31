
#include "examples/ceres_solver/common/quaternion_utils.hpp"

#include <sophus/so3.hpp>

Eigen::Matrix4d quaternionLeftMultiplicationMatrix(Eigen::Quaterniond const & quaternion) {
  // clang-format off
  /**
   * L(q) = [qw·I + [qv]x, qv]
   *        [       -qv^T, qw]
   */
  // clang-format on
  Eigen::Matrix4d L;
  L.block<3, 3>(0, 0) =
      quaternion.w() * Eigen::Matrix3d::Identity() + Sophus::SO3d::hat(quaternion.vec());
  L.block<3, 1>(0, 3) = quaternion.vec();
  L.block<1, 3>(3, 0) = -quaternion.vec().transpose();
  L(3, 3) = quaternion.w();
  return L;
}

Eigen::Matrix4d quaternionRightMultiplicationMatrix(Eigen::Quaterniond const & quaternion) {
  // clang-format off
  /**
   * R(q) = [qw·I - [qv]x, qv]
   *        [       -qv^T, qw]
   */
  // clang-format on
  Eigen::Matrix4d R;
  R.block<3, 3>(0, 0) =
      quaternion.w() * Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(quaternion.vec());
  R.block<3, 1>(0, 3) = quaternion.vec();
  R.block<1, 3>(3, 0) = -quaternion.vec().transpose();
  R(3, 3) = quaternion.w();
  return R;
}
