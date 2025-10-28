#pragma once

#include "examples/ceres_solver/factors/utils.h"

template <typename T>
bool EvaluateRightOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
                        Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
                        Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
                        Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj,
                        Eigen::Quaternion<T> const & i_qm_j, Eigen::Matrix<T, 3, 1> const & i_tm_ij,
                        Eigen::Matrix<T, 6, 6> sqrt_info,
                        Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) {
  Eigen::Matrix<T, 6, 1> error;
  Eigen::Quaternion<T> const j_qe_r{r_qe_j.inverse()};
  Eigen::Quaternion<T> const j_qe_i{j_qe_r * r_qe_i};
  error.template head<3>() = Sophus::SO3<T>{j_qe_i * i_qm_j}.log();
  error.template tail<3>() = j_qe_i * i_tm_ij + j_qe_r * (r_te_ri - r_te_rj);
  whitened_error = sqrt_info * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwi;
      de_dwi.setZero();
      // d/dwi e_w = Jr^{-1}(e_w)·i_R_j^{-1}
      de_dwi.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(error.template head<3>()) *
          i_qm_j.inverse().toRotationMatrix();
      // d/dwi e_t = -r_R_j^{-1}·r_R_i·[i_t_ij]x
      de_dwi.template block<3, 3>(3, 0) = j_qe_i * Sophus::SO3<T>::hat(-i_tm_ij);

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwi{jacobians[0]};
      dr_dwi = sqrt_info * de_dwi;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dti;
      // d/dti e_w = 0
      de_dti.setZero();
      // d/dti e_t = r_R_j^{-1}
      de_dti.template block<3, 3>(3, 0) = j_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dti{jacobians[1]};
      dr_dti = sqrt_info * de_dti;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwj;
      de_dwj.setZero();
      // d/dwj e_w = -Jl^{-1}(e_w)
      de_dwj.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>());
      // d/dwj e_t = [e_t]x
      de_dwj.template block<3, 3>(3, 0) = Sophus::SO3<T>::hat(error.template tail<3>());

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwj{jacobians[2]};
      dr_dwj = sqrt_info * de_dwj;
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtj;
      // d/dtj e_w = 0
      de_dtj.setZero();
      // d/dtj e_t = -r_R_j^{-1}
      de_dtj.template block<3, 3>(3, 0) = -j_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtj{jacobians[3]};
      dr_dtj = sqrt_info * de_dtj;
    }
  }
  return true;
}

template <typename T>
bool EvaluateLeftOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
                       Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
                       Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
                       Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj,
                       Eigen::Quaternion<T> const & i_qm_j, Eigen::Matrix<T, 3, 1> const & i_tm_ij,
                       Eigen::Matrix<T, 6, 6> sqrt_info,
                       Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) {
  Eigen::Quaternion<T> const j_qe_r{r_qe_j.inverse()};
  Eigen::Quaternion<T> const i_qe_r{r_qe_i.inverse()};
  Eigen::Matrix<T, 6, 1> error;
  // e_w = Log(i_R_j·r_R_j^{-1}·r_R_i)
  error.template head<3>() = Sophus::SO3<T>{i_qm_j * j_qe_r * r_qe_i}.log();
  // e_t = i_R_j·r_R_j^{-1}·(r_t_ri - r_t_rj) + i_t_ij
  error.template tail<3>() = i_qm_j * j_qe_r * (r_te_ri - r_te_rj) + i_tm_ij;

  whitened_error = sqrt_info * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwi;
      // d/dwi e_t = 0
      de_dwi.setZero();
      // d/dwi e_w = Jr^{-1}(e_w)·r_R_i^{-1}
      de_dwi.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          i_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwi{jacobians[0]};
      dr_dwi = sqrt_info * de_dwi;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dti;
      // d/dti e_w = 0
      de_dti.setZero();
      // d/dti e_t = i_R_j·r_R_j^{-1}
      de_dti.template block<3, 3>(3, 0) = (i_qm_j * r_qe_j.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dti{jacobians[1]};
      dr_dti = sqrt_info * de_dti;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwj;
      de_dwj.setZero();
      // d/dwj e_w = -Jr^{-1}(e_w)·r_R_i^{-1}
      de_dwj.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          i_qe_r.toRotationMatrix();
      // d/dwj e_t = i_R_j·r_R_j^{-1}·[r_t_ri - r_t_rj]x
      de_dwj.template block<3, 3>(3, 0) =
          (i_qm_j * r_qe_j.inverse()).toRotationMatrix() * Sophus::SO3<T>::hat(r_te_ri - r_te_rj);

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwj{jacobians[2]};
      dr_dwj = sqrt_info * de_dwj;
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtj;
      // d/dtj e_w = 0
      de_dtj.setZero();
      // d/dtj e_t = -i_R_j·r_R_j^{-1}
      de_dtj.template block<3, 3>(3, 0) = -(i_qm_j * r_qe_j.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtj{jacobians[3]};
      dr_dtj = sqrt_info * de_dtj;
    }
  }

  return true;
}
