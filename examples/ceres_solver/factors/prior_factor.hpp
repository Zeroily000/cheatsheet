#include <sophus/so3.hpp>
#include <utility>

#include "examples/ceres_solver/common/quaternion_utils.h"

template <RotationUpdateMode mode>
PriorFunctor<mode>::PriorFunctor(Eigen::Quaterniond r_qm_i, Eigen::Vector3d r_tm_ri,
                                 Eigen::Matrix<double, 6, 6> sqrt_info)
    : r_qm_i_{std::move(r_qm_i)}, r_tm_ri_{std::move(r_tm_ri)}, sqrt_info_{std::move(sqrt_info)} {}

template <RotationUpdateMode mode>
PriorFunctor<mode>::~PriorFunctor() = default;

template <>
template <typename T>
bool PriorFunctor<RotationUpdateMode::kRight>::operator()(T const * const parameter0,
                                                          T const * const parameter1,
                                                          T * const residuals,
                                                          T ** const jacobians) const {
  Eigen::Map<Eigen::Quaternion<T> const> const r_qe_i{parameter0};
  Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_ri{parameter1};
  Eigen::Map<Eigen::Matrix<T, 6, 1>> whitened_error{residuals};

  Eigen::Matrix<T, 6, 1> error;
  // ew = Log(r_R_i^{-1}·R)
  Eigen::Quaternion<T> const i_qe_r{r_qe_i.inverse()};
  error.template head<3>() = Sophus::SO3<T>{i_qe_r * r_qm_i_.cast<T>()}.log();
  // et = r_R_i^{-1}·(t - r_t_ri)
  error.template tail<3>() = i_qe_r * (r_tm_ri_.cast<T>() - r_te_ri);
  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    Eigen::Matrix<T, 6, 6> const dr_de{sqrt_info_.cast<T>()};
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dw;
      // dew/dw = -Jl^{-1}(ew)
      de_dw.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>());
      // det/dw = [et]x
      de_dw.template block<3, 3>(3, 0) = Sophus::SO3<T>::hat(error.template tail<3>());
      // dr/dq = dr/de·de/dw·dw/dq
      Eigen::Matrix<T, 3, 4> const dw_dq{QuaternionRightUpdateJacobianInverse(r_qe_i)};
      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dq{jacobians[0]};
      dr_dq = dr_de * de_dw * dw_dq;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dt;
      // dew/dt = 0
      de_dt.template block<3, 3>(0, 0).setZero();
      // det/dt = -r_R_i^{-1}
      de_dt.template block<3, 3>(3, 0) = -i_qe_r.toRotationMatrix();
      // dr/dt = dr/de·de/dt
      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dt{jacobians[1]};
      dr_dt = dr_de * de_dt;
    }
  }
  return true;
}

template <>
template <typename T>
bool PriorFunctor<RotationUpdateMode::kLeft>::operator()(T const * const parameter0,
                                                         T const * const parameter1,
                                                         T * const residuals,
                                                         T ** const jacobians) const {
  Eigen::Map<Eigen::Quaternion<T> const> const r_qe_i{parameter0};
  Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_ri{parameter1};
  Eigen::Map<Eigen::Matrix<T, 6, 1>> whitened_error{residuals};

  Eigen::Matrix<T, 6, 1> error;
  // ew = Log(R·r_R_i^{-1})
  Eigen::Quaternion<T> const e_q{r_qm_i_.cast<T>() * r_qe_i.inverse()};
  error.template head<3>() = Sophus::SO3<T>{e_q}.log();
  // et = t - R·r_R_i^{-1}·r_t_ri
  error.template tail<3>() = r_tm_ri_.cast<T>() - e_q * r_te_ri;
  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    Eigen::Matrix<T, 6, 6> const dr_de{sqrt_info_.cast<T>()};
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dw;
      // dew/dw = -Jr^{-1}(ew)
      de_dw.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>());
      // det/dw = -R·r_R_i^{-1}·[r_t_ri]x
      de_dw.template block<3, 3>(3, 0) = -(e_q * Sophus::SO3<T>::hat(r_te_ri));
      // dr/dq = dr/de·de/dw·dw/dq
      Eigen::Matrix<T, 3, 4> const dw_dq{QuaternionLeftUpdateJacobianInverse(r_qe_i)};
      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dq{jacobians[0]};
      dr_dq = dr_de * de_dw * dw_dq;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dt;
      // dew/dt = 0
      de_dt.template block<3, 3>(0, 0).setZero();
      // det/dt = -R·r_R_i^{-1}
      de_dt.template block<3, 3>(3, 0) = -e_q.toRotationMatrix();
      // dr/dt = dr/de·de/dt
      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dt{jacobians[1]};
      dr_dt = dr_de * de_dt;
    }
  }
  return true;
}

template <RotationUpdateMode mode>
PriorFactor<mode>::PriorFactor(Eigen::Quaterniond const & r_qm_i, Eigen::Vector3d const & r_tm_ri,
                               Eigen::Matrix<double, 6, 6> const & sqrt_info)
    : functor_{r_qm_i, r_tm_ri, sqrt_info} {}

template <RotationUpdateMode mode>
PriorFactor<mode>::~PriorFactor() = default;

template <RotationUpdateMode mode>
bool PriorFactor<mode>::Evaluate(double const * const * const parameters, double * const residuals,
                                 double ** const jacobians) const {
  return functor_(parameters[0], parameters[1], residuals, jacobians);
}

template <RotationUpdateMode mode>
ceres::CostFunction * PriorFactor<mode>::Create(Eigen::Quaterniond const & r_qm_i,
                                                Eigen::Vector3d const & r_tm_ri,
                                                Eigen::Matrix<double, 6, 6> const & sqrt_info,
                                                JacobianComputationMethod const & method) {
  if (method == JacobianComputationMethod::kAutomatic) {
    return new ceres::AutoDiffCostFunction<PriorFunctor<mode>, 6, 4, 3>{
        new PriorFunctor<mode>{r_qm_i, r_tm_ri, sqrt_info}};
  }
  if (method == JacobianComputationMethod::kAnalytic) {
    return new PriorFactor<mode>{r_qm_i, r_tm_ri, sqrt_info};
  }
  return nullptr;
}
