#pragma once

#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/quaternion_utils.hpp"
#include "examples/ceres_solver/common/rotation_manifold.h"

template <RotationUpdateMode mode>
RotationManifold<mode>::RotationManifold() = default;

template <RotationUpdateMode mode>
RotationManifold<mode>::~RotationManifold() = default;

template <RotationUpdateMode mode>
int RotationManifold<mode>::AmbientSize() const {
  return 4;
}

template <RotationUpdateMode mode>
int RotationManifold<mode>::TangentSize() const {
  return 3;
}

template <>
bool RotationManifold<RotationUpdateMode::kRight>::Plus(double const * const x,
                                                        double const * const delta,
                                                        double * const x_plus_delta) const {
  Eigen::Map<Eigen::Quaterniond const> const q0{x};
  Eigen::Map<Eigen::Vector3d const> const dw{delta};
  Eigen::Map<Eigen::Quaterniond> q1{x_plus_delta};
  q1 = (Sophus::SO3d{q0} * Sophus::SO3d::exp(dw)).unit_quaternion();
  return true;
}

template <>
bool RotationManifold<RotationUpdateMode::kLeft>::Plus(double const * const x,
                                                       double const * const delta,
                                                       double * const x_plus_delta) const {
  Eigen::Map<Eigen::Quaterniond const> const q0{x};
  Eigen::Map<Eigen::Vector3d const> const dw{delta};
  Eigen::Map<Eigen::Quaterniond> q1{x_plus_delta};
  q1 = (Sophus::SO3d::exp(dw) * Sophus::SO3d{q0}).unit_quaternion();
  return true;
}

template <>
bool RotationManifold<RotationUpdateMode::kRight>::PlusJacobian(double const * const x,
                                                                double * const jacobian) const {
  /**
   * q0·dq ≈ q0·(dw/2, 1)
   *       = L(q0)·[dw^T/2, 1]^T
   *       = L(q0)·[I, 0]^T·dw + c
   *
   * J = 1/2·L(q0)·[I, 0]^T
   */
  Eigen::Map<Eigen::Quaterniond const> const q{x};
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> dq_dw{jacobian};
  dq_dw = quaternionLeftMultiplicationMatrix(q).block<4, 3>(0, 0) * .5;
  return true;
}

template <>
bool RotationManifold<RotationUpdateMode::kLeft>::PlusJacobian(double const * const x,
                                                               double * const jacobian) const {
  /**
   * dq·q0 ≈ (dw/2, 1)·q0
   *       = R(q0)·[dw^T/2, 1]^T
   *       = R(q0)·[I, 0]^T·dw + c
   *
   * J = 1/2·R(q0)·[I, 0]^T
   */
  Eigen::Map<Eigen::Quaterniond const> const q{x};
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> dq_dw{jacobian};
  dq_dw = quaternionRightMultiplicationMatrix(q).block<4, 3>(0, 0) * .5;
  return true;
}

template <>
bool RotationManifold<RotationUpdateMode::kRight>::Minus(double const * const y,
                                                         double const * const x,
                                                         double * const y_minus_x) const {
  Eigen::Map<Eigen::Quaterniond const> const q1{y};
  Eigen::Map<Eigen::Quaterniond const> const q0{x};
  Eigen::Map<Eigen::Vector3d> dw{y_minus_x};
  dw = Sophus::SO3d{q0.inverse() * q1}.log();
  return true;
}

template <>
bool RotationManifold<RotationUpdateMode::kLeft>::Minus(double const * const y,
                                                        double const * const x,
                                                        double * const y_minus_x) const {
  Eigen::Map<Eigen::Quaterniond const> const q1{y};
  Eigen::Map<Eigen::Quaterniond const> const q0{x};
  Eigen::Map<Eigen::Vector3d> dw{y_minus_x};
  dw = Sophus::SO3d{q1 * q0.inverse()}.log();
  return true;
}

template <>
bool RotationManifold<RotationUpdateMode::kRight>::MinusJacobian(double const * x,
                                                                 double * jacobian) const {
  /**
   * Let q0^{-1}·q = dq, dq = (dqv, dqw) and Log(dq) = dw, then
   *
   * Log(q0^{-1}·q) = Log(dq)
   *                = |dw|·dqv/sin(|dw|/2)
   *                ≈ 2·dqv
   *                = 2·[I, 0]·dq
   *                = 2·[I, 0]·(q0^{-1}·q)
   *                = 2·[I, 0]·L(q0^{-1})·q
   *
   * J = 2·[I, 0]·L(q0^{-1})
   */
  Eigen::Map<Eigen::Quaterniond const> const q{x};
  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> dw_dq{jacobian};
  dw_dq = quaternionLeftMultiplicationMatrix(q.inverse()).block<3, 4>(0, 0) * 2.;
  return true;
}

template <>
bool RotationManifold<RotationUpdateMode::kLeft>::MinusJacobian(double const * x,
                                                                double * jacobian) const {
  /**
   * Let q·q0^{-1} = dq, dq = (dqv, dqw) and Log(dq) = dw, then
   *
   * Log(q·q0^{-1}) = Log(dq)
   *                = |dw|·dqv/sin(|dw|/2)
   *                ≈ 2·dqv
   *                = 2·[I, 0]·dq
   *                = 2·[I, 0]·(q·q0^{-1})
   *                = 2·[I, 0]·R(q0^{-1})·q
   *
   * J = 2·[I, 0]·R(q0^{-1})
   */
  Eigen::Map<Eigen::Quaterniond const> const q{x};
  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> dw_dq{jacobian};
  dw_dq = quaternionRightMultiplicationMatrix(q.inverse()).block<3, 4>(0, 0) * 2.;
  return true;
}
