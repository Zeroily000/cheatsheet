#pragma once

#include "examples/ceres_solver/factors/between_factor.h"

template <RotationUpdateMode mode>
BetweenFactor<mode>::BetweenFactor(Eigen::Quaterniond a_q_b, Eigen::Vector3d a_t_ab,
                                   Eigen::Matrix<double, 6, 6> sqrt_info)
    : a_q_b_{std::move(a_q_b)}, a_t_ab_{std::move(a_t_ab)}, sqrt_info_{std::move(sqrt_info)} {}

template <RotationUpdateMode mode>
BetweenFactor<mode>::~BetweenFactor() = default;

template <RotationUpdateMode mode>
bool BetweenFactor<mode>::Evaluate(double const * const * parameters, double * residuals,
                                   double ** jacobians) const {
  Eigen::Map<Eigen::Quaterniond const> const r_qe_a{parameters[0]};
  Eigen::Map<Eigen::Vector3d const> const r_te_ra{parameters[1]};
  Eigen::Map<Eigen::Quaterniond const> const r_qe_b{parameters[2]};
  Eigen::Map<Eigen::Vector3d const> const r_te_rb{parameters[3]};
  Eigen::Map<Eigen::Matrix<double, 6, 1>> whitened_error{residuals};
  return Evaluate(r_qe_a, r_te_ra, r_qe_b, r_te_rb, whitened_error, jacobians);
}

template <RotationUpdateMode mode>
template <typename T>
bool BetweenFactor<mode>::Evaluate(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
                                   Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
                                   Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
                                   Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
                                   Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error,
                                   T ** jacobians) const {
  Eigen::Quaternion<T> const a_qm_b{a_q_b_.cast<T>()};
  Eigen::Matrix<T, 3, 1> const a_tm_ab{a_t_ab_.cast<T>()};
  Eigen::Matrix<T, 6, 6> const sqrt_info{sqrt_info_.cast<T>()};
  if (mode == RotationUpdateMode::kRight) {
      return EvaluateRightOplus(r_qe_a, r_te_ra, r_qe_b, r_te_rb, a_qm_b, a_tm_ab, sqrt_info,
                                whitened_error, jacobians);
    }
    if (mode == RotationUpdateMode::kLeft) {
      return EvaluateLeftOplus(r_qe_a, r_te_ra, r_qe_b, r_te_rb, a_qm_b, a_tm_ab, sqrt_info,
                               whitened_error, jacobians);
    }
}
