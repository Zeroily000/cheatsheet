#include <utility>
#include <Sophus/so3.hpp

#include "examples/ceres_solver/factors/prior_factor.h"

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
  Eigen::Quaternion<T> const i_qe_r{r_qe_i.inverse()};
  error.template head<3>(0) = Sophus::SO3{i_qe_r * r_qm_i_.cast<T>()};
  error.template tail<3>(0) = i_qe_r * (r_tm_ri_.cast<T>() - r_te_ri);
  whitened_error = sqrt_info_.cast<T>() * error;

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr) {

    }
  }
}
