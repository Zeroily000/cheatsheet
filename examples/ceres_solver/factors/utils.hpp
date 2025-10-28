#include "examples/ceres_solver/factors/utils.h"

template <typename T>
bool EvaluateRightOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
                        Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
                        Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
                        Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
                        Eigen::Quaternion<T> const & a_qm_b, Eigen::Matrix<T, 3, 1> const & a_tm_ab,
                        Eigen::Matrix<T, 6, 6> sqrt_info,
                        Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) {
  Eigen::Matrix<T, 6, 1> error;
  Eigen::Quaternion<T> const b_qe_r{r_qe_b.inverse()};
  Eigen::Quaternion<T> const b_qe_a{b_qe_r * r_qe_a};
  error.template head<3>() = Sophus::SO3<T>{b_qe_a * a_qm_b}.log();
  error.template tail<3>() = b_qe_a * a_tm_ab + b_qe_r * (r_te_ra - r_te_rb);
  whitened_error = sqrt_info * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwa;
      de_dwa.setZero();
      // d/dwa e_w = Jr^{-1}(e_w)·a_R_b^{-1}
      de_dwa.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(error.template head<3>()) *
          a_qm_b.inverse().toRotationMatrix();
      // d/dwa e_t = -r_R_b^{-1}·r_R_a·[a_t_ab]x
      de_dwa.template block<3, 3>(3, 0) = b_qe_a * Sophus::SO3<T>::hat(-a_tm_ab);

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwa{jacobians[0]};
      dr_dwa = sqrt_info * de_dwa;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dta;
      // d/dta e_w = 0
      de_dta.setZero();
      // d/dta e_t = r_R_b^{-1}
      de_dta.template block<3, 3>(3, 0) = b_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dta{jacobians[1]};
      dr_dta = sqrt_info * de_dta;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwb;
      de_dwb.setZero();
      // d/dwb e_w = -Jl^{-1}(e_w)
      de_dwb.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(error.template head<3>());
      // d/dwb e_t = [e_t]x
      de_dwb.template block<3, 3>(3, 0) = Sophus::SO3<T>::hat(error.template tail<3>());

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwb{jacobians[2]};
      dr_dwb = sqrt_info * de_dwb;
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtb;
      // d/dtb e_w = 0
      de_dtb.setZero();
      // d/dtb e_t = -r_R_b^{-1}
      de_dtb.template block<3, 3>(3, 0) = -b_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtb{jacobians[3]};
      dr_dtb = sqrt_info * de_dtb;
    }
  }
  return true;
}

template <typename T>
static bool EvaluateLeftOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
                              Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
                              Eigen::Quaternion<T> const & a_qm_b,
                              Eigen::Matrix<T, 3, 1> const & a_tm_ab,
                              Eigen::Matrix<T, 6, 6> sqrt_info,
                              Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) {
  Eigen::Quaternion<T> const b_qe_r{r_qe_b.inverse()};
  Eigen::Quaternion<T> const a_qe_r{r_qe_a.inverse()};
  Eigen::Matrix<T, 6, 1> error;
  // e_w = Log(a_R_b·r_R_b^{-1}·r_R_a)
  error.template head<3>() = Sophus::SO3<T>{a_qm_b * b_qe_r * r_qe_a}.log();
  // e_t = a_R_b·r_R_b^{-1}·(r_t_ra - r_t_rb) + a_t_ab
  error.template tail<3>() = a_qm_b * b_qe_r * (r_te_ra - r_te_rb) + a_tm_ab;

  whitened_error = sqrt_info * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwa;
      // d/dw e_t = 0
      de_dwa.setZero();
      // d/dwa e_w = Jr^{-1}(e_w)·r_R_a^{-1}
      de_dwa.template block<3, 3>(0, 0) =
          Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          a_qe_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwa{jacobians[0]};
      dr_dwa = sqrt_info * de_dwa;
    }
    if (jacobians[1] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dta;
      // d/dta e_w = 0
      de_dta.setZero();
      // d/dta e_t = a_R_b·r_R_b^{-1}
      de_dta.template block<3, 3>(3, 0) = (a_qm_b * r_qe_b.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dta{jacobians[1]};
      dr_dta = sqrt_info * de_dta;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Matrix<T, 6, 4> de_dwb;
      de_dwb.setZero();
      // d/dwb e_w = -Jr^{-1}(e_w)·r_R_a^{-1}
      de_dwb.template block<3, 3>(0, 0) =
          -Sophus::SO3<T>::leftJacobianInverse(-error.template head<3>()) *
          a_qe_r.toRotationMatrix();
      // d/dwb e_t = a_R_b·r_R_b^{-1}·[r_t_ra - r_t_rb]x
      de_dwb.template block<3, 3>(3, 0) =
          (a_qm_b * r_qe_b.inverse()).toRotationMatrix() * Sophus::SO3<T>::hat(r_te_ra - r_te_rb);

      Eigen::Map<Eigen::Matrix<T, 6, 4, Eigen::RowMajor>> dr_dwb{jacobians[2]};
      dr_dwb = sqrt_info * de_dwb;
    }
    if (jacobians[3] != nullptr) {
      Eigen::Matrix<T, 6, 3> de_dtb;
      // d/dtb e_w = 0
      de_dtb.setZero();
      // d/dtb e_t = -a_R_b·r_R_b^{-1}
      de_dtb.template block<3, 3>(3, 0) = -(a_qm_b * r_qe_b.inverse()).toRotationMatrix();

      Eigen::Map<Eigen::Matrix<T, 6, 3, Eigen::RowMajor>> dr_dtb{jacobians[3]};
      dr_dtb = sqrt_info * de_dtb;
    }
  }

  return true;
}
