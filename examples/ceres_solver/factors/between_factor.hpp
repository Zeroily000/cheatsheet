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

 private:
  template <typename T>
  bool EvaluateRightOplus(T const * const * parameters, T * residuals, T ** jacobians) const;

  template <typename T>
  bool EvaluateLeftOplus(T const * const * parameters, T * residuals, T ** jacobians) const;

  RotationManifold const * const rotation_manifold_;

  // Rotation from frame b to frame a
  Eigen::Quaterniond a_q_b_;
  // Translation from frame a to frame b, represented in a.
  Eigen::Vector3d a_t_ab_;

  Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info_;
};
