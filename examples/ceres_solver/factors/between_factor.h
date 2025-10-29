#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/rotation_manifold.hpp"
#include "examples/ceres_solver/factors/utils.hpp"

template <RotationUpdateMode mode>
class BetweenFactor : public ceres::SizedCostFunction<6, 4, 3, 4, 3> {
 public:
  BetweenFactor(Eigen::Quaterniond i_qm_j, Eigen::Vector3d i_tm_ij,
                Eigen::Matrix<double, 6, 6> sqrt_info);

  ~BetweenFactor() override;

  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override;

  template <typename T>
  static bool ComputeResidual(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
                              Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj,
                              Eigen::Quaternion<T> const & i_qm_j,
                              Eigen::Matrix<T, 3, 1> const & i_tm_ij,
                              Eigen::Matrix<T, 6, 6> sqrt_info,
                              Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error,
                              T ** jacobians);

 private:
  // Rotation from frame j to frame i.
  Eigen::Quaterniond i_qm_j_;
  // Translation from frame i to frame j, represented in i.
  Eigen::Vector3d i_tm_ij_;

  Eigen::Matrix<double, 6, 6> sqrt_info_;
};
