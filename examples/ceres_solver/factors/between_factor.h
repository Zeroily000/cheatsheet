#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/rotation_manifold.hpp"
#include "examples/ceres_solver/factors/utils.hpp"

template <RotationUpdateMode mode>
class BetweenFactor : public ceres::SizedCostFunction<6, 4, 3, 4, 3> {
 public:
  BetweenFactor(Eigen::Quaterniond a_q_b, Eigen::Vector3d a_t_ab,
                Eigen::Matrix<double, 6, 6> sqrt_info);

  ~BetweenFactor() override;

  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override;

  template <typename T>
  bool Evaluate(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
                Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
                Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
                Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
                Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) const;

  // template <typename T>
  // bool Evaluate(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
  //               Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
  //               Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
  //               Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
  //               Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians) const {
  //   Eigen::Quaternion<T> const a_qm_b{a_q_b_.cast<T>()};
  //   Eigen::Matrix<T, 3, 1> const a_tm_ab{a_t_ab_.cast<T>()};
  //   Eigen::Matrix<T, 6, 6> const sqrt_info{sqrt_info_.cast<T>()};
  //   if (mode == RotationUpdateMode::kRight) {
  //     return EvaluateRightOplus(r_qe_a, r_te_ra, r_qe_b, r_te_rb, a_qm_b, a_tm_ab, sqrt_info,
  //                               whitened_error, jacobians);
  //   }
  //   if (mode == RotationUpdateMode::kLeft) {
  //     return EvaluateLeftOplus(r_qe_a, r_te_ra, r_qe_b, r_te_rb, a_qm_b, a_tm_ab, sqrt_info,
  //                              whitened_error, jacobians);
  //   }
  //   return false;
  // }

 private:
  // Rotation from frame b to frame a
  Eigen::Quaterniond a_q_b_;
  // Translation from frame a to frame b, represented in a.
  Eigen::Vector3d a_t_ab_;

  Eigen::Matrix<double, 6, 6> sqrt_info_;
};
