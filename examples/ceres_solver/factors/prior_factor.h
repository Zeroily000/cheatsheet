#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "examples/ceres_solver/common/types.h"

template <RotationUpdateMode mode>
class PriorFunctor {
 public:
  PriorFunctor(Eigen::Quaterniond r_qm_i, Eigen::Vector3d r_tm_ri,
               Eigen::Matrix<double, 6, 6> sqrt_info);
  ~PriorFunctor();

  /**
   * @brief This function computes the residual of a pose prior and the Jacobian w.r.t the rotation
   * and translation vector.
   *
   * If the rotation parameters use the right update mode, i.e., q' = q * Exp(dw), residuals are
   *
   * e = [r_R_i^{-1}, -r_R_i^{-1}·r_t_ri]·[R, t]
   *     [         0,                  1] [0, 1]
   *
   * ew = Log(r_R_i^{-1}·R)
   * et = r_R_i^{-1}·(t - r_t_ri)
   *
   * and Jacobians are
   *
   * dew/dw = d/dw Log((r_R_i·Exp(dw))^{-1}·R)
   *        = d/dw Log(Exp(-dw)·r_R_i^{-1}·R)
   *        = d/dw Log(Exp(-dw)·Exp(ew))
   *        = d/dw Log(Exp(ew - Jl^{-1}(ew)·dw))
   *        = d/dw (ew - Jl^{-1}(ew)·dw)
   *        = -Jl^{-1}(ew)
   *
   * det/dw = d/dw (Exp(-dw)·r_R_i^{-1}·(t - r_t_ri))
   *        = d/dw (Exp(-dw)·et)
   *        = d/dw ((I - [dw]x)·et)
   *        = d/dw [et]x·dw
   *        = [et]x
   *
   * dew/dt = 0
   *
   * det/dt = d/dt r_R_i^{-1}·(t - r_t_ri)
   *        = -r_R_i^{-1}
   *
   * If the rotation parameters use the left update mode, i.e., q' = Exp(dw) * q, residuals are
   *
   * e = [R, t]·[r_R_i^{-1}, -r_R_i^{-1}·r_t_ri]
   *     [0, 1] [         0,                  1]
   *
   * ew = Log(R·r_R_i^{-1})
   * et = t - R·r_R_i^{-1}·r_t_ri
   *
   * and Jacobians are
   *
   * dew/dw = d/dw Log(R·(r_R_i·Exp(dw))^{-1})
   *        = d/dw Log(R·Exp(-dw)·r_R_i^{-1})
   *        = d/dw Log(Exp(-R·dw)·R·r_R_i^{-1})
   *        = d/dw Log(Exp(-R·dw)·Exp(ew))
   *        = d/dw Log(Exp(ew - Jl^{-1}(ew)R·dw))
   *        = d/dw (ew - Jl^{-1}(ew)·R·dw)
   *        = -Jl^{-1}(ew)·R
   *
   * det/dw = d/dw (t - R·Exp(-dw)·r_R_i^{-1}·r_t_ri)
   *        = d/dw (-Exp(-R·dw)·R·r_R_i^{-1}·r_t_ri)
   *        = d/dw (-(I - [R·dw]x)·R·r_R_i^{-1}·r_t_ri)
   *        = d/dw ([R·dw]x·R·r_R_i^{-1}·r_t_ri)
   *        = d/dw -[R·r_R_i^{-1}·r_t_ri]x·R·dw
   *        = -[R·r_R_i^{-1}·r_t_ri]x·R
   *
   * dew/dt = 0
   *
   * det/dt = d/dt (t - R·r_R_i^{-1}·r_t_ri)
   *        = -R·r_R_i^{-1}
   *
   * @tparam T The scalar type, e.g., double or ceres::Jet.
   * @param[in] parameter0 The rotation parameter as a unit quaternion (x, y, z, w).
   * @param[in] parameter1 The translation parameter as a 3D vector.
   * @param[out] residuals The whitened error as a 6D vector.
   * @param[out] jacobians The derivatives of the residuals w.r.t the parameters.
   * @return true.
   */
  template <typename T>
  bool operator()(T const * parameter0, T const * parameter1, T * residuals,
                  T ** jacobians = nullptr) const;

 private:
  Eigen::Quaterniond r_qm_i_;
  Eigen::Vector3d r_tm_ri_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

template <RotationUpdateMode mode>
class PriorFactor : public ceres::SizedCostFunction<6, 4, 3> {
 public:
  PriorFactor(Eigen::Quaterniond const & r_qm_i, Eigen::Vector3d const & r_tm_ri,
              Eigen::Matrix<double, 6, 6> const & sqrt_info);

  ~PriorFactor() override;

  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override;

  static ceres::CostFunction * Create(Eigen::Quaterniond const & r_qm_i,
                                      Eigen::Vector3d const & r_tm_ri,
                                      Eigen::Matrix<double, 6, 6> const & sqrt_info,
                                      JacobianComputationMethod const & method);

 private:
  PriorFunctor<mode> functor_;
};

#include "examples/ceres_solver/factors/prior_factor.hpp"
