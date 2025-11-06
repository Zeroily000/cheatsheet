#pragma once

#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/quaternion_utils.hpp"
#include "examples/ceres_solver/factors/between_factor.h"

template <RotationUpdateMode mode>
BetweenFunctor<mode>::BetweenFunctor(Eigen::Quaterniond i_qm_j, Eigen::Vector3d i_tm_ij,
                                     Eigen::Matrix<double, 6, 6> sqrt_info)
    : i_qm_j_{std::move(i_qm_j)}, i_tm_ij_{std::move(i_tm_ij)}, sqrt_info_{std::move(sqrt_info)} {}

template <RotationUpdateMode mode>
BetweenFunctor<mode>::~BetweenFunctor() = default;

template <>
template <typename T>
bool BetweenFunctor<RotationUpdateMode::kRight>::operator()(
    T const * const parameter0, T const * const parameter1, T const * const parameter2,
    T const * const parameter3, T * const residuals, T ** const jacobians) const {
  Eigen::Map<Eigen::Quaternion<T> const> const r_qe_i{parameter0};
  Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_ri{parameter1};
  Eigen::Map<Eigen::Quaternion<T> const> const r_qe_j{parameter2};
  Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_rj{parameter3};
  Eigen::Map<Eigen::Matrix<T, 6, 1>> whitened_error{residuals};

  Eigen::Matrix<T, 6, 1> error;
  Eigen::Quaternion<T> const j_qe_r{r_qe_j.inverse()};
  Eigen::Quaternion<T> const j_qe_i{j_qe_r * r_qe_i};
  error.template head<3>() = Sophus::SO3<T>{j_qe_i * i_qm_j_.cast<T>()}.log();
  error.template tail<3>() = j_qe_i * i_tm_ij_.cast<T>() + j_qe_r * (r_te_ri - r_te_rj);
  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwi;
      // d/dwi e_w = Jr^{-1}(e_w)·i_R_j^{-1}
      de_dwi.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(error.template head<3>()) *
          i_qm_j_.cast<T>().inverse().toRotationMatrix();
      // d/dwi e_t = -r_R_j^{-1}·r_R_i·[i_t_ij]x
      de_dwi.template block<3, 3>(3, 0) = j_qe_i * Sophus::SO3<T>::hat(-i_tm_ij_.cast<T>());

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwi{jacobians[0]};
      dr_dwi = sqrt_info_.cast<T>() * de_dwi * QuaternionRightUpdateJacobianInverse(r_qe_i);
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dti;
      // d/dti e_w = 0
      de_dti.template block<3, 3>(0, 0).setZero();
      // d/dti e_t = r_R_j^{-1}
      de_dti.template block<3, 3>(3, 0) = j_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dti{jacobians[1]};
      dr_dti = sqrt_info_.cast<T>() * de_dti;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwj;
      // d/dwj e_w = -Jl^{-1}(e_w)
      de_dwj.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>());
      // d/dwj e_t = [e_t]x
      de_dwj.template block<3, 3>(3, 0) = Sophus::SO3<T>::hat(error.template tail<3>());

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwj{jacobians[2]};
      dr_dwj = sqrt_info_.cast<T>() * de_dwj * QuaternionRightUpdateJacobianInverse(r_qe_j);
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtj;
      // d/dtj e_w = 0
      de_dtj.template block<3, 3>(0, 0).setZero();
      // d/dtj e_t = -r_R_j^{-1}
      de_dtj.template block<3, 3>(3, 0) = -j_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtj{jacobians[3]};
      dr_dtj = sqrt_info_.cast<T>() * de_dtj;
    }
  }
  return true;
}

template <>
template <typename T>
bool BetweenFunctor<RotationUpdateMode::kLeft>::operator()(
    T const * const parameter0, T const * const parameter1, T const * const parameter2,
    T const * const parameter3, T * const residuals, T ** const jacobians) const {
  Eigen::Map<Eigen::Quaternion<T> const> const r_qe_i{parameter0};
  Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_ri{parameter1};
  Eigen::Map<Eigen::Quaternion<T> const> const r_qe_j{parameter2};
  Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_rj{parameter3};
  Eigen::Map<Eigen::Matrix<T, 6, 1>> whitened_error{residuals};

  Eigen::Quaternion<T> const j_qe_r{r_qe_j.inverse()};
  Eigen::Quaternion<T> const i_qe_r{r_qe_i.inverse()};
  Eigen::Matrix<T, 6, 1> error;
  // e_w = Log(i_R_j·r_R_j^{-1}·r_R_i)
  error.template head<3>() = Sophus::SO3<T>{i_qm_j_.cast<T>() * j_qe_r * r_qe_i}.log();
  // e_t = i_R_j·r_R_j^{-1}·(r_t_ri - r_t_rj) + i_t_ij
  error.template tail<3>() = i_qm_j_.cast<T>() * j_qe_r * (r_te_ri - r_te_rj) + i_tm_ij_.cast<T>();

  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwi;
      // d/dwi e_w = Jr^{-1}(e_w)·r_R_i^{-1}
      de_dwi.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          i_qe_r.toRotationMatrix();
      // d/dwi e_t = 0
      de_dwi.template block<3, 3>(3, 0).setZero();

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwi{jacobians[0]};
      dr_dwi = sqrt_info_.cast<T>() * de_dwi * QuaternionLeftUpdateJacobianInverse(r_qe_i);
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dti;
      // d/dti e_w = 0
      de_dti.template block<3, 3>(0, 0).setZero();
      // d/dti e_t = i_R_j·r_R_j^{-1}
      de_dti.template block<3, 3>(3, 0) = (i_qm_j_.cast<T>() * r_qe_j.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dti{jacobians[1]};
      dr_dti = sqrt_info_.cast<T>() * de_dti;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwj;
      // d/dwj e_w = -Jr^{-1}(e_w)·r_R_i^{-1}
      de_dwj.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          i_qe_r.toRotationMatrix();
      // d/dwj e_t = i_R_j·r_R_j^{-1}·[r_t_ri - r_t_rj]x
      de_dwj.template block<3, 3>(3, 0) =
          (i_qm_j_.cast<T>() * r_qe_j.inverse()).toRotationMatrix() *
          Sophus::SO3<T>::hat(r_te_ri - r_te_rj);

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwj{jacobians[2]};
      dr_dwj = sqrt_info_.cast<T>() * de_dwj * QuaternionLeftUpdateJacobianInverse(r_qe_j);
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtj;
      // d/dtj e_w = 0
      de_dtj.template block<3, 3>(0, 0).setZero();
      // d/dtj e_t = -i_R_j·r_R_j^{-1}
      de_dtj.template block<3, 3>(3, 0) =
          -(i_qm_j_.cast<T>() * r_qe_j.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtj{jacobians[3]};
      dr_dtj = sqrt_info_.cast<T>() * de_dtj;
    }
  }

  return true;
}

template <RotationUpdateMode mode>
BetweenFactor<mode>::BetweenFactor(Eigen::Quaterniond const & i_qm_j,
                                   Eigen::Vector3d const & i_tm_ij,
                                   Eigen::Matrix<double, 6, 6> const & sqrt_info)
    : functor_{i_qm_j, i_tm_ij, sqrt_info} {}

template <RotationUpdateMode mode>
BetweenFactor<mode>::~BetweenFactor() = default;

template <RotationUpdateMode mode>
bool BetweenFactor<mode>::Evaluate(double const * const * const parameters,
                                   double * const residuals, double ** const jacobians) const {
  return functor_(parameters[0], parameters[1], parameters[2], parameters[3], residuals, jacobians);
}

template <RotationUpdateMode mode>
ceres::CostFunction * BetweenFactor<mode>::Create(Eigen::Quaterniond const & i_qm_j,
                                                  Eigen::Vector3d const & i_tm_ij,
                                                  Eigen::Matrix<double, 6, 6> const & sqrt_info,
                                                  JacobianComputationMethod const & method) {
  if (method == JacobianComputationMethod::kAutomatic) {
    return new ceres::AutoDiffCostFunction<BetweenFunctor<mode>, 6, 4, 3, 4, 3>{
        new BetweenFunctor<mode>{i_qm_j, i_tm_ij, sqrt_info}};
  }
  if (method == JacobianComputationMethod::kAnalytic) {
    return new BetweenFactor<mode>{i_qm_j, i_tm_ij, sqrt_info};
  }
  return nullptr;
}

// template <RotationUpdateMode mode>
// bool BetweenFunctor<mode>::operator()(T const * parameter0, T const * parameter1,
//                                      T const * parameter2, T const * parameter3, T * residuals,
//                                      T ** jacobians = nullptr) const {
//   Eigen::Map<Eigen::Quaterniond const> const r_qe_i{parameters[0]};
//   Eigen::Map<Eigen::Vector3d const> const r_te_ri{parameters[1]};
//   Eigen::Map<Eigen::Quaterniond const> const r_qe_j{parameters[2]};
//   Eigen::Map<Eigen::Vector3d const> const r_te_rj{parameters[3]};
//   Eigen::Map<Eigen::Matrix<double, 6, 1>> whitened_error{residuals};
//   return Evaluate(r_qe_i, r_te_ri, r_qe_j, r_te_rj, i_qm_j_, i_tm_ij_, sqrt_info_,
//   whitened_error,
//                   jacobians);
// }
