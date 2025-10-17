#include "examples/ceres_solver/common/pose3_manifolds.hpp"

#include <sophus/so3.hpp>

Pose3AnalyticDiffManifold::Pose3AnalyticDiffManifold() = default;

Pose3AnalyticDiffManifold::~Pose3AnalyticDiffManifold() = default;

int Pose3AnalyticDiffManifold::AmbientSize() const { return kAmbientSize; }

int Pose3AnalyticDiffManifold::TangentSize() const { return kTangentSize; }

bool Pose3AnalyticDiffManifold::Plus(double const * const x, double const * const delta,
                                     double * const x_plus_delta) const {
  Eigen::Map<Eigen::Quaterniond const> const q0{x + kAmbientRotationIndex};
  Eigen::Map<Eigen::Vector3d const> const t0{x + kAmbientTranslationIndex};

  Eigen::Map<Eigen::Vector3d const> const dw{delta + kTangentRotationIndex};
  Eigen::Map<Eigen::Vector3d const> const dt{delta + kTangentTranslationIndex};

  Eigen::Map<Eigen::Quaterniond> q1{x_plus_delta + kAmbientRotationIndex};
  Eigen::Map<Eigen::Vector3d> t1{x_plus_delta + kAmbientTranslationIndex};

  q1 = q0 * Sophus::SO3d::exp(dw).unit_quaternion();
  t1 = t0 + dt;

  return true;
}

bool Pose3AnalyticDiffManifold::PlusJacobian(double const * const /* x */,
                                             double * const jacobian) const {
  Eigen::Map<Eigen::Matrix<double, kAmbientSize, kTangentSize, Eigen::RowMajor>> dummy{jacobian};
  dummy.setZero();
  dummy.block<kTangentSize, kTangentSize>(0, 0).setIdentity();

  return true;
}

// bool Pose3AnalyticDiffManifold::RightMultiplyByPlusJacobian(double const * const x,
//                                                             int const num_rows,
//                                                             double const * const ambient_matrix,
//                                                             double * const tangent_matrix) const
//                                                             {
//   return false;
// }

bool Pose3AnalyticDiffManifold::Minus(double const * const y, double const * const x,
                                      double * const y_minus_x) const {
  Eigen::Map<Eigen::Quaterniond const> const q1{y + kAmbientRotationIndex};
  Eigen::Map<Eigen::Vector3d const> const t1{y + kAmbientTranslationIndex};

  Eigen::Map<Eigen::Quaterniond const> const q0{x + kAmbientRotationIndex};
  Eigen::Map<Eigen::Vector3d const> const t0{x + kAmbientTranslationIndex};

  Eigen::Map<Eigen::Vector3d> dw{y_minus_x + kTangentRotationIndex};
  Eigen::Map<Eigen::Vector3d> dt{y_minus_x + kTangentTranslationIndex};

  dw = Sophus::SO3d{q0.inverse() * q1}.log();
  dt = q0.inverse() * (t1 - t0);

  return true;
}

bool Pose3AnalyticDiffManifold::MinusJacobian(double const * x, double * jacobian) const {
  return false;
}
