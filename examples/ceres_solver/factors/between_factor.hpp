#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "examples/ceres_solver/common/rotation_manifold.hpp"

class BetweenFactor : public ceres::SizedCostFunction<6, 4, 3, 4, 3> {
 public:
  static std::size_t constexpr kResidualRotationSize{3};
  static std::size_t constexpr kResidualTranslationSize{3};

  static std::size_t constexpr kResidualSize{kResidualRotationSize + kResidualTranslationSize};

  BetweenFactor(RotationManifold const * rotation_manifold, Eigen::Quaterniond a_q_b,
                Eigen::Vector3d a_t_b,
                Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info);
  ~BetweenFactor() override;

  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override;

 private:
  bool RightOplusResidual(double const * const * parameters, double * residuals,
                          double ** jacobians) const;

  bool LeftOplusResidual(double const * const * parameters, double * residuals,
                         double ** jacobians) const;

  RotationManifold const * const rotation_manifold_;
  Eigen::Quaterniond a_q_b_;
  Eigen::Vector3d a_t_b_;
  Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info_;
};
