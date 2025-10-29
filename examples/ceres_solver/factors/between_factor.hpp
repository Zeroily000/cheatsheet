#pragma once

#include "examples/ceres_solver/factors/between_factor.h"

template <RotationUpdateMode mode>
BetweenFactor<mode>::BetweenFactor(Eigen::Quaterniond i_qm_j, Eigen::Vector3d i_tm_ij,
                                   Eigen::Matrix<double, 6, 6> sqrt_info)
    : i_qm_j_{std::move(i_qm_j)}, i_tm_ij_{std::move(i_tm_ij)}, sqrt_info_{std::move(sqrt_info)} {}

template <RotationUpdateMode mode>
BetweenFactor<mode>::~BetweenFactor() = default;

template <RotationUpdateMode mode>
bool BetweenFactor<mode>::Evaluate(double const * const * parameters, double * residuals,
                                   double ** jacobians) const {
  Eigen::Map<Eigen::Quaterniond const> const r_qe_i{parameters[0]};
  Eigen::Map<Eigen::Vector3d const> const r_te_ri{parameters[1]};
  Eigen::Map<Eigen::Quaterniond const> const r_qe_j{parameters[2]};
  Eigen::Map<Eigen::Vector3d const> const r_te_rj{parameters[3]};
  Eigen::Map<Eigen::Matrix<double, 6, 1>> whitened_error{residuals};
  return ComputeResidual(r_qe_i, r_te_ri, r_qe_j, r_te_rj, i_qm_j_, i_tm_ij_, sqrt_info_,
                         whitened_error, jacobians);
}

template <>
template <typename T>
bool BetweenFactor<RotationUpdateMode::kRight>::ComputeResidual(
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj, Eigen::Quaternion<T> const & i_qm_j,
    Eigen::Matrix<T, 3, 1> const & i_tm_ij, Eigen::Matrix<T, 6, 6> sqrt_info,
    Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) {
  return EvaluateRightOplus(r_qe_i, r_te_ri, r_qe_j, r_te_rj, i_qm_j, i_tm_ij, sqrt_info,
                            whitened_error, jacobians);
}

template <>
template <typename T>
bool BetweenFactor<RotationUpdateMode::kLeft>::ComputeResidual(
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj, Eigen::Quaternion<T> const & i_qm_j,
    Eigen::Matrix<T, 3, 1> const & i_tm_ij, Eigen::Matrix<T, 6, 6> sqrt_info,
    Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) {
  return EvaluateLeftOplus(r_qe_i, r_te_ri, r_qe_j, r_te_rj, i_qm_j, i_tm_ij, sqrt_info,
                            whitened_error, jacobians);
}
