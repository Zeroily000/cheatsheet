#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class BetweenFactor : public ceres::SizedCostFunction<6, 4, 3, 4, 3> {
 public:
  static std::size_t constexpr kResidualRotationSize{3};
  static std::size_t constexpr kResidualTranslationSize{3};

  static std::size_t constexpr kResidualSize{kResidualRotationSize + kResidualTranslationSize};

  BetweenFactor(Eigen::Quaterniond a_R_b, Eigen::Vector3d a_t_b,
                Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info);
  ~BetweenFactor() override;

  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override;

 private:
  Eigen::Quaterniond a_R_b_;
  Eigen::Vector3d a_t_b_;
  Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info_;
};
