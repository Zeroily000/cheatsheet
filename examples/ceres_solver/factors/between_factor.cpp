#include "examples/ceres_solver/factors/between_factor.hpp"

#include <sophus/so3.hpp>
#include <utility>

BetweenFactor::BetweenFactor(RotationManifold const * const rotation_manifold,
                             Eigen::Quaterniond a_q_b, Eigen::Vector3d a_t_ab,
                             Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info)
    : rotation_manifold_{rotation_manifold},
      a_q_b_{std::move(a_q_b)},
      a_t_ab_{std::move(a_t_ab)},
      sqrt_info_{std::move(sqrt_info)} {}

BetweenFactor::~BetweenFactor() = default;

bool BetweenFactor::Evaluate(double const * const * const parameters, double * const residuals,
                             double ** const jacobians) const {
  Eigen::Map<Eigen::Quaterniond const> const r_qe_a{parameters[0]};
  Eigen::Map<Eigen::Vector3d const> const r_te_ra{parameters[1]};
  Eigen::Map<Eigen::Quaterniond const> const r_qe_b{parameters[2]};
  Eigen::Map<Eigen::Vector3d const> const r_te_rb{parameters[3]};
  Eigen::Map<Eigen::Matrix<double, kResidualSize, 1>> whitened_error{residuals};
  if (rotation_manifold_->GetMode() == RotationManifold::Mode::kRightPerturbation) {
    return EvaluateRightOplus(r_qe_a, r_te_ra, r_qe_b, r_te_rb, a_q_b_, a_t_ab_, sqrt_info_,
                              whitened_error, jacobians);
  }
  if (rotation_manifold_->GetMode() == RotationManifold::Mode::kLeftPerturbation) {
    return EvaluateLeftOplus(r_qe_a, r_te_ra, r_qe_b, r_te_rb, a_q_b_, a_t_ab_, sqrt_info_,
                              whitened_error, jacobians);
  }

  return false;
}

template <typename T>
bool BetweenFactor::EvaluateRightOplus(
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb, Eigen::Quaternion<T> const & a_qm_b,
    Eigen::Matrix<T, 3, 1> const & a_tm_ab,
    Eigen::Matrix<T, kResidualSize, kResidualSize> sqrt_info,
    Eigen::Map<Eigen::Matrix<T, kResidualSize, 1>> & whitened_error, T ** jacobians) {
  Eigen::Matrix<T, kResidualSize, 1> error;
  Eigen::Quaternion<T> const b_qe_r{r_qe_b.inverse()};
  Eigen::Quaternion<T> const b_qe_a{b_qe_r * r_qe_a};
  error.template head<3>() = Sophus::SO3<T>{b_qe_a * a_qm_b}.log();
  error.template tail<3>() = b_qe_a * a_tm_ab + b_qe_r * (r_te_ra - r_te_rb);
  whitened_error = sqrt_info * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      // clang-format off
        /**
         * d/dwa e_w = d/dwa Log(r_R_b^{-1}·r_R_a·Exp(wa)·a_R_b)
         *           = d/dwa Log(r_R_b^{-1}·r_R_a·Exp(wa)·a_R_b)
         *           = d/dwa Log(r_R_b^{-1}·r_R_a·a_R_b·Exp(a_R_b^{-1}·wa))
         *           = d/dwa Log(e_R·Exp(a_R_b^{-1}·wa))
         *           = d/dwa Log(Exp(e_w + Jr^{-1}(e_w)·(a_R_b^{-1}·wa)))
         *           = Jr^{-1}(e_w)·a_R_b^{-1}
         *
         * d/dwa e_t = d/dwa (r_R_b^{-1}·r_R_a·Exp(wa)·a_t_ab
         *           = d/dwa (r_R_b^{-1}·r_R_a·(I + [wa]x)·a_t_ab
         *           = d/dwa (r_R_b^{-1}·r_R_a·[wa]x·a_t_ab)
         *           = d/dwa -(r_R_b^{-1}·r_R_a·[a_t_ab]x·wa)
         *           = -r_R_b^{-1}·r_R_a·[a_t_ab]x
         */
      // clang-format on
      Eigen::Matrix<T, kResidualSize, 4> de_dwa;
      de_dwa.setZero();
      de_dwa.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(error.template head<3>()) *
          a_qm_b.inverse().toRotationMatrix();
      de_dwa.template block<3, 3>(3, 0) = b_qe_a * Sophus::SO3<T>::hat(-a_tm_ab);

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 4, Eigen::RowMajor>> dr_dwa{jacobians[0]};
      dr_dwa = sqrt_info * de_dwa;
    }
    if (jacobians[1] != nullptr) {
      // clang-format off
        /**
         * d/dta e_w = 0
         *
         * d/dta e_t = r_R_b^{-1}
         */
      // clang-format on
      Eigen::Matrix<T, kResidualSize, 3> de_dta;
      de_dta.setZero();
      de_dta.template block<3, 3>(3, 0) = b_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 3, Eigen::RowMajor>> dr_dta{jacobians[1]};
      dr_dta = sqrt_info * de_dta;
    }
    if (jacobians[2] != nullptr) {
      // clang-format off
        /**
         * d/dwb e_w = d/dwb Log((r_R_b·Exp(wb))^{-1}·r_R_a·a_R_b)
         *           = d/dwb Log(Exp(-wb)·r_R_b^{-1}·r_R_a·a_R_b)
         *           = d/dwb Log(Exp(-wb)·e_R)
         *           = d/dwb Log(Exp(e_w - Jl^{-1}(e_w)·wb))
         *           = -Jl^{-1}(e_w)
         *
         * d/dwb e_t = d/dwb (r_R_b·Exp(wb))^{-1}·(r_R_a·a_t_ab + r_t_ra - r_t_rb)
         *           = d/dwb Exp(-wb)·r_R_b ^{-1}·(r_R_a·a_t_ab + r_t_ra - r_t_rb)
         *           = d/dwb (I - [w]x)·e_t
         *           = d/dwb -[w]x·e_t
         *           = [e_t]x
         */
      // clang-format on
      Eigen::Matrix<T, kResidualSize, 4> de_dwb;
      de_dwb.setZero();
      de_dwb.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>());
      de_dwb.template block<3, 3>(3, 0) = Sophus::SO3<T>::hat(error.template tail<3>());

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 4, Eigen::RowMajor>> dr_dwb{jacobians[2]};
      dr_dwb = sqrt_info * de_dwb;
    }
    if (jacobians[3] != nullptr) {
      // clang-format off
        /**
         * d/dtb e_w = 0
         *
         * d/dtb e_t = -r_R_b^{-1}
         */
      // clang-format on
      Eigen::Matrix<T, kResidualSize, 3> de_dtb;
      de_dtb.setZero();
      de_dtb.template block<3, 3>(3, 0) = -b_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 3, Eigen::RowMajor>> dr_dtb{jacobians[3]};
      dr_dtb = sqrt_info * de_dtb;
    }
  }
  return true;
}

template <typename T>
bool BetweenFactor::EvaluateLeftOplus(
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb, Eigen::Quaternion<T> const & a_qm_b,
    Eigen::Matrix<T, 3, 1> const & a_tm_ab,
    Eigen::Matrix<T, kResidualSize, kResidualSize> sqrt_info,
    Eigen::Map<Eigen::Matrix<T, kResidualSize, 1>> & whitened_error, T ** jacobians) {
  // clang-format off
  /**
   * a_T_b·e = z
   * e·r_T_a^{-1}·r_T_b = z
   *
   * e = z·r_T_b^{-1}·r_T_a
   *
   *   = [a_R_b, a_t_ab]·[r_R_b^{-1}, -r_R_b^{-1}·r_t_rb]·[r_R_a, r_t_ra]
   *     [    0,      1] [         0,                  1] [   0,      1]
   *
   *   = [a_R_b·r_R_b^{-1}·r_R_a, a_R_b·r_R_b^{-1}·(r_t_ra - r_t_rb) + a_t_ab]
   *     [                     0,                                           1]
   *
   * e_w = Log(a_R_b·r_R_b^{-1}·r_R_a)
   * e_t = a_R_b·r_R_b^{-1}·(r_t_ra - r_t_rb) + a_t_ab
   */
  // clang-format on
  Eigen::Quaternion<T> const b_qe_r{r_qe_b.inverse()};
  Eigen::Quaternion<T> const a_qe_r{r_qe_a.inverse()};
  Eigen::Matrix<T, kResidualSize, 1> error;
  error.template head<3>() = Sophus::SO3<T>{a_qm_b * b_qe_r * r_qe_a}.log();
  error.template tail<3>() = a_qm_b * b_qe_r * (r_te_ra - r_te_rb) + a_tm_ab;

  whitened_error = sqrt_info * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      // clang-format off
      /**
       * d/dwa e_w = d/dwa Log(a_R_b·r_R_b^{-1}·Exp(wa)·r_R_a)
       *           = d/dwa Log(a_R_b·r_R_b^{-1}·r_R_a·Exp(r_R_a^{-1}·wa)
       *           = d/dwa Log(e_R·Exp(r_R_a^{-1}·wa))
       *           = d/dwa Log(Exp(e_w + Jr^{-1}(e_w)·(r_R_a^{-1}·wa))
       *           = Jr^{-1}(e_w)·r_R_a^{-1}
       *
       * d/dw e_t = 0
       */
      // clang-format on

      Eigen::Matrix<T, kResidualSize, 4> de_dwa;
      de_dwa.setZero();
      de_dwa.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          a_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 4, Eigen::RowMajor>> dr_dwa{jacobians[0]};
      dr_dwa = sqrt_info * de_dwa;
    }
    if (jacobians[1] != nullptr) {
      // clang-format off
      /**
       * d/dta e_w = 0
       *
       * d/dta e_t = a_R_b·r_R_b^{-1}
       */
      // clang-format on
      Eigen::Matrix<T, kResidualSize, 3> de_dta;
      de_dta.setZero();
      de_dta.template block<3, 3>(3, 0) = (a_qm_b * r_qe_b.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 3, Eigen::RowMajor>> dr_dta{jacobians[1]};
      dr_dta = sqrt_info * de_dta;
    }
    if (jacobians[2] != nullptr) {
      // clang-format off
      /**
       * d/dwb e_w = d/dwb Log(a_R_b·(Exp(wb)·r_R_b)^{-1}·r_R_a)
       *           = d/dwb Log(a_R_b·r_R_b^{-1}·Exp(-wb)·r_R_a)
       *           = d/dwb Log(a_R_b·r_R_b^{-1}·r_R_a·Exp(-r_R_a^{-1}·wb))
       *           = d/dwb Log(e_R·Exp(-r_R_a^{-1}·wb))
       *           = d/dwb Log(Exp(e_w - Jr^{-1}(e_w)·r_R_a^{-1}·wb))
       *           = -Jr^{-1}(e_w)·r_R_a^{-1}
       *
       * d/dwb e_t = d/dwb a_R_b·(Exp(wb)·r_R_b)^{-1}·(r_t_ra - r_t_rb)
       *           = d/dwb a_R_b·r_R_b^{-1}·Exp(-wb)·(r_t_ra - r_t_rb)
       *           = d/dwb a_R_b·r_R_b^{-1}·(I - [wb]x)·(r_t_ra - r_t_rb)
       *           = d/dwb a_R_b·r_R_b^{-1}·[-wb]x·(r_t_ra - r_t_rb)
       *           = d/dwb a_R_b·r_R_b^{-1}·[r_t_ra - r_t_rb]x·wb
       *           = a_R_b·r_R_b^{-1}·[r_t_ra - r_t_rb]x
       */
      // clang-format on
      Eigen::Matrix<T, kResidualSize, 4> de_dwb;
      de_dwb.setZero();
      de_dwb.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          a_qe_r.toRotationMatrix();
      de_dwb.template block<3, 3>(3, 0) =
          (a_qm_b * r_qe_b.inverse()).toRotationMatrix() * Sophus::SO3<T>::hat(r_te_ra - r_te_rb);

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 4, Eigen::RowMajor>> dr_dwb{jacobians[2]};
      dr_dwb = sqrt_info * de_dwb;
    }
    if (jacobians[3] != nullptr) {
      // clang-format off
      /**
       * d/dt e_w = 0
       *
       * d/dt e_t = -a_R_b·r_R_b^{-1}
       */
      // clang-format on
      Eigen::Matrix<T, kResidualSize, 3> de_dtb;
      de_dtb.setZero();
      de_dtb.template block<3, 3>(3, 0) = -(a_qm_b * r_qe_b.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, kResidualSize, 3, Eigen::RowMajor>> dr_dtb{jacobians[3]};
      dr_dtb = sqrt_info * de_dtb;
    }
  }

  return true;
}
