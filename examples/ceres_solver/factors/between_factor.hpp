#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/rotation_manifold.hpp"

class BetweenFactor : public ceres::SizedCostFunction<6, 4, 3, 4, 3> {
 public:
  static std::size_t constexpr kResidualRotationSize{3};
  static std::size_t constexpr kResidualTranslationSize{3};

  static std::size_t constexpr kResidualSize{kResidualRotationSize + kResidualTranslationSize};

  BetweenFactor(RotationManifold const * rotation_manifold, Eigen::Quaterniond a_q_b,
                Eigen::Vector3d a_t_ab,
                Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info);
  ~BetweenFactor() override;

  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override;

  template <typename T>
  static bool EvaluateRightOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
                                 Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
                                 Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
                                 Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
                                 Eigen::Quaternion<T> const & a_qm_b,
                                 Eigen::Matrix<T, 3, 1> const & a_tm_ab,
                                 Eigen::Matrix<T, kResidualSize, kResidualSize> sqrt_info,
                                 Eigen::Map<Eigen::Matrix<T, kResidualSize, 1>> & whitened_error,
                                 T ** jacobians);

  template <typename T>
  static bool EvaluateLeftOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
                                Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
                                Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
                                Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
                                Eigen::Quaternion<T> const & a_qm_b,
                                Eigen::Matrix<T, 3, 1> const & a_tm_ab,
                                Eigen::Matrix<T, kResidualSize, kResidualSize> sqrt_info,
                                Eigen::Map<Eigen::Matrix<T, kResidualSize, 1>> & whitened_error,
                                T ** jacobians);

 private:

  RotationManifold const * const rotation_manifold_;

  // Rotation from frame b to frame a
  Eigen::Quaterniond a_q_b_;
  // Translation from frame a to frame b, represented in a.
  Eigen::Vector3d a_t_ab_;

  Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info_;
};
