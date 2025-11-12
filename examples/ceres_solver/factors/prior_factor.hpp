#include <Sophus/so3.hpp>
#include <utility>

// #include "examples/ceres_solver/factors/prior_factor.h"

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

  /**
   * e = [r_R_i^{-1}, -r_R_i^{-1}·r_t_ri]·[R, t]
   *     [         0,                  1] [0, 1]
   *
   * ew = Log(r_R_i^{-1}·R)
   * et = r_R_i^{-1}·(t - r_t_ri)
   */
  Eigen::Matrix<T, 6, 1> error;
  Eigen::Quaternion<T> const i_qe_r{r_qe_i.inverse()};
  error.template head<3>(0) = Sophus::SO3{i_qe_r * r_qm_i_.cast<T>()}.log();
  error.template tail<3>(0) = i_qe_r * (r_tm_ri_.cast<T>() - r_te_ri);
  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dw;
      /**
       * d/dw ew = d/dw Log((r_R_i·Exp(dw))^{-1}·R)
       *         = d/dw Log(Exp(-dw)·r_R_i^{-1}·R)
       *         = d/dw Log(Exp(-dw)·Exp(ew))
       *         = d/dw Log(Exp(ew - Jl^{-1}(ew)·dw))
       *         = d/dw (ew - Jl^{-1}(ew)·dw)
       *         = -Jl^{-1}(ew)
       */
      de_dw.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>(0));
      /**
       * d/dw et = d/dw (Exp(-dw)·r_R_i^{-1}·(t - r_t_ri))
       *         = d/dw (Exp(-dw)·et)
       *         = d/dw ((I - [dw]x)·et)
       *         = d/dw [et]x·dw
       *         = [et]x
       */
      de_dw.template block<3, 3>(3, 0) = Sophus::SO3<T>::hat(error.template tail<3>(0));

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dq{jacobians[0]};
      dr_dq = sqrt_info_.cast<T>() * dr_dw * QuaternionRightUpdateJacobianInverse(r_qe_i);
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dt;
      /**
       * d/dt ew = 0
       */
      de_dt.template block<3, 3>(0, 0).setZero();
      /**
       * d/dt et = d/dt r_R_i^{-1}·(t - r_t_ri)
       *         = -r_R_i^{-1}
       */
      de_dt.template block<3, 3>(3, 0) = -i_qe_r.toRotationMatrix();
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

  /**
   * e = [R, t]·[r_R_i^{-1}, -r_R_i^{-1}·r_t_ri]
   *     [0, 1] [         0,                  1]
   *
   * ew = Log(R·r_R_i^{-1})
   * et = t - R·r_R_i^{-1}·r_t_ri
   */
  Eigen::Matrix<T, 6, 1> error;
  Eigen::Quaternion<T> const i_qe_r{r_qe_i.inverse()};
  Eigen::Quaternion<T> const e_q{r_qm_i_.cast<T>() * i_qe_r};
  error.template head<3>(0) = Sophus::SO3{e_q}.log();
  error.template tail<3>(0) = r_tm_ri_.cast<T>() - e_q * r_te_ri;
  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dw;
      /**
       * d/dw ew = d/dw Log(R·(r_R_i·Exp(dw))^{-1})
       *         = d/dw Log(R·Exp(-dw)·r_R_i^{-1})
       *         = d/dw Log(Exp(-R·dw)·R·r_R_i^{-1})
       *         = d/dw Log(Exp(-R·dw)·Exp(ew))
       *         = d/dw Log(Exp(ew - Jl^{-1}(ew)R·dw))
       *         = d/dw (ew - Jl^{-1}(ew)·R·dw)
       *         = -Jl^{-1}(ew)·R
       */
      de_dw.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>(0)) *
          r_qm_i_.cast<T>().toRotationMatrix();
      /**
       * d/dw et = d/dw (t - R·Exp(-dw)·r_R_i^{-1}·r_t_ri)
       *         = d/dw (-Exp(-R·dw)·R·r_R_i^{-1}·r_t_ri)
       *         = d/dw (-(I - [R·dw]x)·R·r_R_i^{-1}·r_t_ri)
       *         = d/dw ([R·dw]x·R·r_R_i^{-1}·r_t_ri)
       *         = d/dw -[R·r_R_i^{-1}·r_t_ri]x·R·dw
       *         = -[R·r_R_i^{-1}·r_t_ri]x·R
       */
      de_dw.template block<3, 3>(3, 0) =
          Sophus::SO3<T>::hat(e_q * r_te_ri) * r_qm_i_.cast<T>().toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dq{jacobians[0]};
      dr_dq = sqrt_info_.cast<T>() * dr_dw * QuaternionLeftUpdateJacobianInverse(r_qe_i);
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dt;
      /**
       * d/dt ew = 0
       */
      de_dt.template block<3, 3>(0, 0).setZero();
      /**
       * d/dt et = d/dt (t - R·r_R_i^{-1}·r_t_ri)
       *         = -R·r_R_i^{-1}
       */
      de_dt.template block<3, 3>(3, 0) = -e_q.toRotationMatrix();
    }
  }
  return true;
}
