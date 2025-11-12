#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/quaternion_utils.h"

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
    Eigen::Matrix<T, 6, 6> const dr_de{sqrt_info_.cast<T>()};
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwi;
      // dew/dwi = Jr^{-1}(ew)·i_R_j^{-1}
      de_dwi.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          i_qm_j_.inverse().toRotationMatrix().cast<T>();
      // det/dwi = -r_R_j^{-1}·r_R_i·[i_t_ij]x
      de_dwi.template block<3, 3>(3, 0) =
          j_qe_i.toRotationMatrix() * Sophus::SO3<T>::hat(-i_tm_ij_.cast<T>());
      // dr/dqi = dr/de·de/dwi·dwi/dqi
      Eigen::Matrix<T, 3, 4> const dwi_dqi{QuaternionRightUpdateJacobianInverse(r_qe_i)};
      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dqi{jacobians[0]};
      dr_dqi = dr_de * de_dwi * dwi_dqi;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dti;
      // dew/dti = 0
      de_dti.template block<3, 3>(0, 0).setZero();
      // det/dti = r_R_j^{-1}
      de_dti.template block<3, 3>(3, 0) = j_qe_r.toRotationMatrix();
      // dr/dti = dr/de·de/dti
      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dti{jacobians[1]};
      dr_dti = dr_de * de_dti;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwj;
      // dew/dwj = -Jl^{-1}(e_w)
      de_dwj.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>());
      // det/dwj = [e_t]x
      de_dwj.template block<3, 3>(3, 0) = Sophus::SO3<T>::hat(error.template tail<3>());
      // dr/dqj = dr/de·de/dwj·dwj/dqj
      Eigen::Matrix<T, 3, 4> const dwj_dqj{QuaternionRightUpdateJacobianInverse(r_qe_j)};
      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dqj{jacobians[2]};
      dr_dqj = dr_de * de_dwj * dwj_dqj;
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtj;
      // dew/dtj = 0
      de_dtj.template block<3, 3>(0, 0).setZero();
      // det/dtj = -r_R_j^{-1}
      de_dtj.template block<3, 3>(3, 0) = -j_qe_r.toRotationMatrix();
      // dr/dtj = dr/de·de/dtj
      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtj{jacobians[3]};
      dr_dtj = dr_de * de_dtj;
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
  // ew = Log(i_R_j·r_R_j^{-1}·r_R_i)
  error.template head<3>() = Sophus::SO3<T>{i_qm_j_.cast<T>() * j_qe_r * r_qe_i}.log();
  // et = i_R_j·r_R_j^{-1}·(r_t_ri - r_t_rj) + i_t_ij
  error.template tail<3>() = i_qm_j_.cast<T>() * j_qe_r * (r_te_ri - r_te_rj) + i_tm_ij_.cast<T>();

  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    Eigen::Matrix<T, 6, 6> const dr_de{sqrt_info_.cast<T>()};
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwi;
      // dew/dwi = Jr^{-1}(e_w)·r_R_i^{-1}
      de_dwi.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          i_qe_r.toRotationMatrix();
      // det/dwi = 0
      de_dwi.template block<3, 3>(3, 0).setZero();
      // dr/dqi = dr/de·de/dwi·dwi/dqi
      Eigen::Matrix<T, 3, 4> const dwi_dqi{QuaternionLeftUpdateJacobianInverse(r_qe_i)};
      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwi{jacobians[0]};
      dr_dwi = dr_de * de_dwi * dwi_dqi;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dti;
      // dew/dti = 0
      de_dti.template block<3, 3>(0, 0).setZero();
      // det/dti = i_R_j·r_R_j^{-1}
      de_dti.template block<3, 3>(3, 0) = (i_qm_j_.cast<T>() * r_qe_j.inverse()).toRotationMatrix();
      // dr/dti = dr/de·de/dti
      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dti{jacobians[1]};
      dr_dti = dr_de * de_dti;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dwj;
      // dew/dwj = -Jr^{-1}(e_w)·r_R_i^{-1}
      de_dwj.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          i_qe_r.toRotationMatrix();
      // det/dwj = i_R_j·r_R_j^{-1}·[r_t_ri - r_t_rj]x
      de_dwj.template block<3, 3>(3, 0) =
          (i_qm_j_.cast<T>() * r_qe_j.inverse()).toRotationMatrix() *
          Sophus::SO3<T>::hat(r_te_ri - r_te_rj);
      // dr/dqj = dr/de·de/dwj·dwj/dqj
      Eigen::Matrix<T, 3, 4> const dwj_dqj{QuaternionLeftUpdateJacobianInverse(r_qe_j)};
      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwj{jacobians[2]};
      dr_dwj = dr_de * de_dwj * dwj_dqj;
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtj;
      // dew/dtj = 0
      de_dtj.template block<3, 3>(0, 0).setZero();
      // det/dtj = -i_R_j·r_R_j^{-1}
      de_dtj.template block<3, 3>(3, 0) =
          -(i_qm_j_.cast<T>() * r_qe_j.inverse()).toRotationMatrix();
      // dr/dtj = dr/de·de/dtj
      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtj{jacobians[3]};
      dr_dtj = dr_de * de_dtj;
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
