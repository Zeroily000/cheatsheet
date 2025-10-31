#pragma once

#include <sophus/so3.hpp>
#include <utility>

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

template <RotationUpdateMode mode>
bool RotationManifold<mode>::PlusJacobian(double const * const /* x */,
                                          double * const jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> dummy{jacobian};
  dummy.setIdentity();
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

template <RotationUpdateMode mode>
bool RotationManifold<mode>::MinusJacobian(double const * x, double * jacobian) const {
  return false;
}
