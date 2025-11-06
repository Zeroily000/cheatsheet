#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "examples/ceres_solver/common/rotation_manifold.hpp"

template <RotationUpdateMode mode>
class PriorFunctor {
 public:
  PriorFunctor(Eigen::Quaterniond r_qm_i, Eigen::Vector3d r_tm_ri,
               Eigen::Matrix<double, 6, 6> sqrt_info);
  ~PriorFunctor();

  template <typename T>
  bool operator()(T const * parameter0, T const * parameter1, T * residuals,
                  T ** jacobians = nullptr) const;

 private:
  Eigen::Quaterniond r_qm_i_;
  Eigen::Vector r_tm_ri_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};
