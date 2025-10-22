#include "examples/ceres_solver/factors/between_factor.hpp"

#include <sophus/so3.hpp>
#include <utility>

BetweenFactor::BetweenFactor(RotationManifold const * const rotation_manifold,
                             Eigen::Quaterniond a_q_b, Eigen::Vector3d a_t_b,
                             Eigen::Matrix<double, kResidualSize, kResidualSize> sqrt_info)
    : rotation_manifold_{rotation_manifold},
      a_q_b_{std::move(a_q_b)},
      a_t_b_{std::move(a_t_b)},
      sqrt_info_{std::move(sqrt_info)} {}

BetweenFactor::~BetweenFactor() = default;

bool BetweenFactor::Evaluate(double const * const * const parameters, double * const residuals,
                             double ** jacobians) const {
  if (rotation_manifold_->GetMode() == RotationManifold::Mode::kRightPerturbation) {
    return RightOplusResidual(parameters, residuals, jacobians);
  }

  return false;
  // Eigen::Map<Eigen::Quaterniond const> const ref_R_a{parameters[0]};
  // Eigen::Map<Eigen::Vector3d const> const ref_t_a{parameters[1]};

  // Eigen::Map<Eigen::Quaterniond const> const ref_R_b{parameters[2]};
  // Eigen::Map<Eigen::Vector3d const> const ref_t_b{parameters[3]};

  // // clang-format off
  // /**
  //  * a_T_b ∙ e = z
  //  * r_T_a^{-1} ∙ r_T_b ∙ e = z
  //  *
  //  * e = r_T_b^{-1} ∙ r_T_a ∙ z
  //  *
  //  *   = [r_R_b^{-1}, -r_R_b^{-1} ∙ r_t_b] ∙ [r_R_a, r_t_a] ∙ [a_R_b, a_t_b]
  //  *     [         0,                   1]   [   0,      1]   [    0,     1]
  //  *
  //  *   = [r_R_b^{-1} ∙ r_R_a ∙ a_R_b, r_R_b^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b)]
  //  *     [                         0,                                                1]
  //  *
  //  * e_w = Log(r_R_b^{-1} ∙ r_R_a ∙ a_R_b)
  //  * e_t = r_R_b^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b)
  //  */
  // // clang-format on
  // Eigen::Quaterniond const b_R_ref{ref_R_b.inverse()};
  // Eigen::Matrix<double, kResidualSize, 1> error;
  // error.head<3>() = Sophus::SO3d{b_R_ref * ref_R_a * a_R_b_}.log();
  // error.tail<3>() = b_R_ref * (ref_R_a * a_t_b_ + ref_t_a - ref_t_b);

  // Eigen::Map<Eigen::Matrix<double, kResidualSize, 1>> res{residuals};
  // res = sqrt_info_ * error;

  // if (jacobians != nullptr) {
  //   if (jacobians[0] != nullptr) {
  //     // clang-format off
  //     /**
  //      * d/dw e_w = lim_{w->0} (Log(r_R_b^{-1} ∙ r_R_a ∙ Exp(w) ∙ a_R_b) - e_w) / w
  //      *          = lim_{w->0} (Log(r_R_b^{-1} ∙ r_R_a ∙ Exp(w) ∙ a_R_b) - e_w) / w
  //      *          = lim_{w->0} (Log(r_R_b^{-1} ∙ r_R_a ∙ a_R_b ∙ Exp(a_R_b^{-1} ∙ w)) - e_w) / w
  //      *          = lim_{w->0} (Log(e_R ∙ Exp(a_R_b^{-1} ∙ w)) - e_w) / w
  //      *          = lim_{w->0} (Log(Exp(e_w + Jr^{-1}(e_w) ∙ (a_R_b^{-1} ∙ w)) - e_w) / w
  //      *          = Jr^{-1}(e_w) ∙ a_R_b^{-1}
  //      *
  //      * d/dw e_t = lim_{w->0} (r_R_b^{-1} ∙ r_R_a ∙ Exp(w) ∙ a_t_b - e_t) / w
  //      *          = lim_{w->0} (r_R_b^{-1} ∙ r_R_a ∙ (I + [w]x) ∙ a_t_b - e_t) / w
  //      *          = lim_{w->0} (r_R_b^{-1} ∙ r_R_a ∙ [w]x ∙ a_t_b) / w
  //      *          = lim_{w->0} -(r_R_b^{-1} ∙ r_R_a ∙ [a_t_b]x ∙ w) / w
  //      *          = -r_R_b^{-1} ∙ r_R_a ∙ [a_t_b]x
  //      */
  //     // clang-format on

  //     Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor> de_dwa;
  //     de_dwa.setZero();
  //     de_dwa.block<3, 3>(0, 0) =
  //         Sophus::SO3d::leftJacobianInverse(error.head<3>()) *
  //         a_R_b_.inverse().toRotationMatrix();
  //     de_dwa.block<3, 3>(3, 0) = b_R_ref * ref_R_a * Sophus::SO3d::hat(-a_t_b_);

  //     Eigen::Map<Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor>> dr_dwa{jacobians[0]};
  //     dr_dwa = sqrt_info_ * de_dwa;
  //   }
  //   if (jacobians[1] != nullptr) {
  //     // clang-format off
  //     /**
  //      * d/dt e_w = 0
  //      *
  //      * d/dt e_t = r_R_b^{-1}
  //      */
  //     // clang-format on
  //     Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor> de_dta;
  //     de_dta.setZero();
  //     de_dta.block<3, 3>(3, 0) = b_R_ref.toRotationMatrix();

  //     Eigen::Map<Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor>> dr_dta{jacobians[1]};
  //     dr_dta = sqrt_info_ * de_dta;
  //   }
  //   if (jacobians[2] != nullptr) {
  //     // clang-format off
  //     /**
  //      * d/dw e_w = lim_{w->0} (Log((r_R_b ∙ Exp(w))^{-1} ∙ r_R_a ∙ a_R_b) - e_w) / w
  //      *          = lim_{w->0} (Log(Exp(-w) ∙ r_R_b^{-1} ∙ r_R_a ∙ a_R_b) - e_w) / w
  //      *          = lim_{w->0} (Log(Exp(-w) ∙ e_R) - e_w) / w
  //      *          = lim_{w->0} (Log(Exp(e_w + Jl^{-1}(e_w) ∙ (-w)) - e_w) / w
  //      *          = -Jl^{-1}(e_w)
  //      *
  //      * d/dw e_t = lim_{w->0} ((r_R_b ∙ Exp(w))^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b) -
  //      e_t) / w
  //      *          = lim_{w->0} (Exp(-w) ∙ r_R_b ^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b) -
  //      e_t) / w
  //      *          = lim_{w->0} ((I - [w]x) ∙ e_t - e_t) / w
  //      *          = lim_{w->0} (-[w]x ∙ e_t) / w
  //      *          = [e_t]x
  //      */
  //     // clang-format on
  //     Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor> de_dwb;
  //     de_dwb.setZero();
  //     de_dwb.block<3, 3>(0, 0) = -Sophus::SO3d::leftJacobianInverse(error.head<3>());
  //     de_dwb.block<3, 3>(3, 0) = Sophus::SO3d::hat(error.tail<3>());

  //     Eigen::Map<Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor>> dr_dwb{jacobians[2]};
  //     dr_dwb = sqrt_info_ * de_dwb;
  //   }
  //   if (jacobians[3] != nullptr) {
  //     // clang-format off
  //     /**
  //      * d/dt e_w = 0
  //      *
  //      * d/dt e_t = -r_R_b^{-1}
  //      */
  //     // clang-format on
  //     Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor> de_dtb;
  //     de_dtb.setZero();
  //     de_dtb.block<3, 3>(3, 0) = -b_R_ref.toRotationMatrix();

  //     Eigen::Map<Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor>> dr_dtb{jacobians[3]};
  //     dr_dtb = sqrt_info_ * de_dtb;
  //   }
  // }

  // return true;
}

bool BetweenFactor::RightOplusResidual(double const * const * const parameters,
                                       double * const residuals, double ** jacobians) const {
  Eigen::Map<Eigen::Quaterniond const> const r_q_a{parameters[0]};
  Eigen::Map<Eigen::Vector3d const> const r_t_a{parameters[1]};

  Eigen::Map<Eigen::Quaterniond const> const r_q_b{parameters[2]};
  Eigen::Map<Eigen::Vector3d const> const r_t_b{parameters[3]};

  // clang-format off
  /**
   * a_T_b ∙ e = z
   * r_T_a^{-1} ∙ r_T_b ∙ e = z
   *
   * e = r_T_b^{-1} ∙ r_T_a ∙ z
   *
   *   = [r_R_b^{-1}, -r_R_b^{-1} ∙ r_t_b] ∙ [r_R_a, r_t_a] ∙ [a_R_b, a_t_b]
   *     [         0,                   1]   [   0,      1]   [    0,     1]
   *
   *   = [r_R_b^{-1} ∙ r_R_a ∙ a_R_b, r_R_b^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b)]
   *     [                         0,                                                1]
   *
   * e_w = Log(r_R_b^{-1} ∙ r_R_a ∙ a_R_b)
   * e_t = r_R_b^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b)
   */
  // clang-format on
  Eigen::Quaterniond const b_q_r{r_q_b.inverse()};
  Eigen::Quaterniond const b_q_a{b_q_r * r_q_a};
  Eigen::Matrix<double, kResidualSize, 1> error;
  error.head<3>() = Sophus::SO3d{b_q_a * a_q_b_}.log();
  error.tail<3>() = b_q_a * a_t_b_ + b_q_r * (r_t_a - r_t_b);

  Eigen::Map<Eigen::Matrix<double, kResidualSize, 1>> res{residuals};
  res = sqrt_info_ * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {
      // clang-format off
      /**
       * d/dw e_w = lim_{w->0} (Log(r_R_b^{-1} ∙ r_R_a ∙ Exp(w) ∙ a_R_b) - e_w) / w
       *          = lim_{w->0} (Log(r_R_b^{-1} ∙ r_R_a ∙ Exp(w) ∙ a_R_b) - e_w) / w
       *          = lim_{w->0} (Log(r_R_b^{-1} ∙ r_R_a ∙ a_R_b ∙ Exp(a_R_b^{-1} ∙ w)) - e_w) / w
       *          = lim_{w->0} (Log(e_R ∙ Exp(a_R_b^{-1} ∙ w)) - e_w) / w
       *          = lim_{w->0} (Log(Exp(e_w + Jr^{-1}(e_w) ∙ (a_R_b^{-1} ∙ w)) - e_w) / w
       *          = Jr^{-1}(e_w) ∙ a_R_b^{-1}
       *
       * d/dw e_t = lim_{w->0} (r_R_b^{-1} ∙ r_R_a ∙ Exp(w) ∙ a_t_b - e_t) / w
       *          = lim_{w->0} (r_R_b^{-1} ∙ r_R_a ∙ (I + [w]x) ∙ a_t_b - e_t) / w
       *          = lim_{w->0} (r_R_b^{-1} ∙ r_R_a ∙ [w]x ∙ a_t_b) / w
       *          = lim_{w->0} -(r_R_b^{-1} ∙ r_R_a ∙ [a_t_b]x ∙ w) / w
       *          = -r_R_b^{-1} ∙ r_R_a ∙ [a_t_b]x
       */
      // clang-format on

      Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor> de_dwa;
      de_dwa.setZero();
      de_dwa.block<3, 3>(0, 0) =
          Sophus::SO3d::leftJacobianInverse(error.head<3>()) * a_q_b_.inverse().toRotationMatrix();
      de_dwa.block<3, 3>(3, 0) = b_q_a * Sophus::SO3d::hat(-a_t_b_);

      Eigen::Map<Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor>> dr_dwa{jacobians[0]};
      dr_dwa = sqrt_info_ * de_dwa;
    }
    if (jacobians[1] != nullptr) {
      // clang-format off
      /**
       * d/dt e_w = 0
       *
       * d/dt e_t = r_R_b^{-1}
       */
      // clang-format on
      Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor> de_dta;
      de_dta.setZero();
      de_dta.block<3, 3>(3, 0) = b_q_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor>> dr_dta{jacobians[1]};
      dr_dta = sqrt_info_ * de_dta;
    }
    if (jacobians[2] != nullptr) {
      // clang-format off
      /**
       * d/dw e_w = lim_{w->0} (Log((r_R_b ∙ Exp(w))^{-1} ∙ r_R_a ∙ a_R_b) - e_w) / w
       *          = lim_{w->0} (Log(Exp(-w) ∙ r_R_b^{-1} ∙ r_R_a ∙ a_R_b) - e_w) / w
       *          = lim_{w->0} (Log(Exp(-w) ∙ e_R) - e_w) / w
       *          = lim_{w->0} (Log(Exp(e_w + Jl^{-1}(e_w) ∙ (-w)) - e_w) / w
       *          = -Jl^{-1}(e_w)
       *
       * d/dw e_t = lim_{w->0} ((r_R_b ∙ Exp(w))^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b) - e_t) / w
       *          = lim_{w->0} (Exp(-w) ∙ r_R_b ^{-1} ∙ (r_R_a ∙ a_t_b + ref_t_a - ref_t_b) - e_t) / w
       *          = lim_{w->0} ((I - [w]x) ∙ e_t - e_t) / w
       *          = lim_{w->0} (-[w]x ∙ e_t) / w
       *          = [e_t]x
       */
      // clang-format on
      Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor> de_dwb;
      de_dwb.setZero();
      de_dwb.block<3, 3>(0, 0) = -Sophus::SO3d::leftJacobianInverse(error.head<3>());
      de_dwb.block<3, 3>(3, 0) = Sophus::SO3d::hat(error.tail<3>());

      Eigen::Map<Eigen::Matrix<double, kResidualSize, 4, Eigen::RowMajor>> dr_dwb{jacobians[2]};
      dr_dwb = sqrt_info_ * de_dwb;
    }
    if (jacobians[3] != nullptr) {
      // clang-format off
      /**
       * d/dt e_w = 0
       *
       * d/dt e_t = -r_R_b^{-1}
       */
      // clang-format on
      Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor> de_dtb;
      de_dtb.setZero();
      de_dtb.block<3, 3>(3, 0) = -b_q_r.toRotationMatrix();

      Eigen::Map<Eigen::Matrix<double, kResidualSize, 3, Eigen::RowMajor>> dr_dtb{jacobians[3]};
      dr_dtb = sqrt_info_ * de_dtb;
    }
  }

  return true;
}
