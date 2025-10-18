#include "examples/ceres_solver/common/rotation_manifold.hpp"

#include <sophus/so3.hpp>

RotationManifold::RotationManifold() = default;

RotationManifold::~RotationManifold() = default;

int RotationManifold::AmbientSize() const { return kAmbientSize; }

int RotationManifold::TangentSize() const { return kTangentSize; }

bool RotationManifold::Plus(double const * const x, double const * const delta,
                            double * const x_plus_delta) const {
  Eigen::Map<Eigen::Quaterniond const> const q0{x};
  Eigen::Map<Eigen::Vector3d const> const dw{delta};
  Eigen::Map<Eigen::Quaterniond> q1{x_plus_delta};

  q1 = q0 * Sophus::SO3d::exp(dw).unit_quaternion();

  return true;
}

bool RotationManifold::PlusJacobian(double const * const /* x */, double * const jacobian) const {
  Eigen::Map<Eigen::Matrix<double, kAmbientSize, kTangentSize, Eigen::RowMajor>> dummy{jacobian};
  dummy.setZero();
  dummy.block<kTangentSize, kTangentSize>(0, 0).setIdentity();

  return true;
}

bool RotationManifold::Minus(double const * const y, double const * const x,
                             double * const y_minus_x) const {
  Eigen::Map<Eigen::Quaterniond const> const q1{y};
  Eigen::Map<Eigen::Quaterniond const> const q0{x};
  Eigen::Map<Eigen::Vector3d> dw{y_minus_x};

  dw = Sophus::SO3d{q0.inverse() * q1}.log();

  return true;
}

bool RotationManifold::MinusJacobian(double const * x, double * jacobian) const {
  return false;
}
