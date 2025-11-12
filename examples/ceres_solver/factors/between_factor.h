#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "examples/ceres_solver/common/types.h"

template <RotationUpdateMode mode>
class BetweenFunctor {
 public:
  BetweenFunctor(Eigen::Quaterniond i_qm_j, Eigen::Vector3d i_tm_ij,
                 Eigen::Matrix<double, 6, 6> sqrt_info);

  ~BetweenFunctor();

  /**
   * @brief This function computes the residual between two poses and the Jacobian w.r.t the
   * rotation and translation vector.
   *
   * If the rotation parameters use the right update mode, i.e., q' = q * Exp(dw), residuals are
   *
   * i_T_j·e = z
   * r_T_i^{-1}·r_T_j·e = z
   * e = r_T_j^{-1}·r_T_i·z
   *   = [r_R_j^{-1}, -r_R_j^{-1}·r_t_rj]·[r_R_i, r_t_ri]·[i_R_j, i_t_ij]
   *     [         0,                  1] [   0,       1] [    0,      1]
   *   = [r_R_j^{-1}·r_R_i·i_R_j, r_R_j^{-1}·r_R_i·i_t_ij + r_R_j^{-1}·(r_t_ri - r_t_rj)]
   *     [                     0,                                                      1]
   *
   * e_w = Log(r_R_j^{-1}·r_R_i·i_R_j)
   * e_t = r_R_j^{-1}·(r_R_i·i_t_ij + r_t_ri - r_t_rj)
   *
   * and Jacobians are
   *
   * dew/dwi = d/dwi Log(r_R_j^{-1}·r_R_i·Exp(wi)·i_R_j)
   *         = d/dwi Log(r_R_j^{-1}·r_R_i·Exp(wi)·i_R_j)
   *         = d/dwi Log(r_R_j^{-1}·r_R_i·i_R_j·Exp(i_R_j^{-1}·wi))
   *         = d/dwi Log(e_R·Exp(i_R_j^{-1}·wi))
   *         = d/dwi Log(Exp(e_w + Jr^{-1}(e_w)·(i_R_j^{-1}·wi)))
   *         = Jr^{-1}(e_w)·i_R_j^{-1}
   *
   * det/dwi = d/dwi (r_R_j^{-1}·r_R_i·Exp(wi)·i_t_ij
   *         = d/dwi (r_R_j^{-1}·r_R_i·(I + [wi]x)·i_t_ij
   *         = d/dwi (r_R_j^{-1}·r_R_i·[wi]x·i_t_ij)
   *         = d/dwi -(r_R_j^{-1}·r_R_i·[i_t_ij]x·wi)
   *         = -r_R_j^{-1}·r_R_i·[i_t_ij]x
   *
   * dew/dti = 0
   *
   * det/dti = r_R_j^{-1}
   *
   * dew/dwj = d/dwj Log((r_R_j·Exp(wj))^{-1}·r_R_i·i_R_j)
   *         = d/dwj Log(Exp(-wj)·r_R_j^{-1}·r_R_i·i_R_j)
   *         = d/dwj Log(Exp(-wj)·e_R)
   *         = d/dwj Log(Exp(e_w - Jl^{-1}(e_w)·wj))
   *         = -Jl^{-1}(e_w)
   *
   * det/dwj = d/dwj (r_R_j·Exp(wj))^{-1}·(r_R_i·i_t_ij + r_t_ri - r_t_rj)
   *         = d/dwj Exp(-wj)·r_R_j ^{-1}·(r_R_i·i_t_ij + r_t_ri - r_t_rj)
   *         = d/dwj (I - [w]x)·e_t
   *         = d/dwj -[w]x·e_t
   *         = [e_t]x
   *
   * dew/dtj = 0
   *
   * det/dtj = -r_R_j^{-1}
   *
   * If the rotation parameters use the left update mode, i.e., q' = Exp(dw) * q, residuals are
   *
   * i_T_j·e = z
   * e·r_T_i^{-1}·r_T_j = z
   *
   * e = z·r_T_j^{-1}·r_T_i
   *   = [i_R_j, i_t_ij]·[r_R_j^{-1}, -r_R_j^{-1}·r_t_rj]·[r_R_i, r_t_ri]
   *     [    0,      1] [         0,                  1] [   0,      1]
   *   = [i_R_j·r_R_j^{-1}·r_R_i, i_R_j·r_R_j^{-1}·(r_t_ri - r_t_rj) + i_t_ij]
   *     [                     0,                                           1]
   *
   * e_w = Log(i_R_j·r_R_j^{-1}·r_R_i)
   * e_t = i_R_j·r_R_j^{-1}·(r_t_ri - r_t_rj) + i_t_ij
   *
   * dew/dwi = d/dwi Log(i_R_j·r_R_j^{-1}·Exp(wi)·r_R_i)
   *         = d/dwi Log(i_R_j·r_R_j^{-1}·r_R_i·Exp(r_R_i^{-1}·wi)
   *         = d/dwi Log(e_R·Exp(r_R_i^{-1}·wi))
   *         = d/dwi Log(Exp(e_w + Jr^{-1}(e_w)·(r_R_i^{-1}·wi))
   *         = Jr^{-1}(e_w)·r_R_i^{-1}
   *
   * det/dwi = 0
   *
   * dew/dti = 0
   *
   * det/dti = i_R_j·r_R_j^{-1}
   *
   * dew/dwj = d/dwj Log(i_R_j·(Exp(wj)·r_R_j)^{-1}·r_R_i)
   *         = d/dwj Log(i_R_j·r_R_j^{-1}·Exp(-wj)·r_R_i)
   *         = d/dwj Log(i_R_j·r_R_j^{-1}·r_R_i·Exp(-r_R_i^{-1}·wj))
   *         = d/dwj Log(e_R·Exp(-r_R_i^{-1}·wj))
   *         = d/dwj Log(Exp(e_w - Jr^{-1}(e_w)·r_R_i^{-1}·wj))
   *         = -Jr^{-1}(e_w)·r_R_i^{-1}
   *
   * det/dwj = d/dwj i_R_j·(Exp(wj)·r_R_j)^{-1}·(r_t_ri - r_t_rj)
   *         = d/dwj i_R_j·r_R_j^{-1}·Exp(-wj)·(r_t_ri - r_t_rj)
   *         = d/dwj i_R_j·r_R_j^{-1}·(I - [wj]x)·(r_t_ri - r_t_rj)
   *         = d/dwj i_R_j·r_R_j^{-1}·[-wj]x·(r_t_ri - r_t_rj)
   *         = d/dwj i_R_j·r_R_j^{-1}·[r_t_ri - r_t_rj]x·wj
   *         = i_R_j·r_R_j^{-1}·[r_t_ri - r_t_rj]x
   *
   * dew/dtj = 0
   *
   * det/dtj = -i_R_j·r_R_j^{-1}
   *
   * @tparam T The scalar type, e.g., double or ceres::Jet.
   * @param[in] parameter0 The rotation parameters of frame i as a unit quaternion (x, y, z, w);
   * @param[in] parameter1 The translation parameters of frame i as a 3D vector.
   * @param[in] parameter2 The rotation parameters of frame j as a unit quaternion (x, y, z, w);
   * @param[in] parameter3 The translation parameters of frame j as a 3D vector.
   * @param[out] residuals The whitened error as a 6D vector.
   * @param[out] jacobians The derivatives of the residuals w.r.t the parameters.
   */
  template <typename T>
  bool operator()(T const * parameter0, T const * parameter1, T const * parameter2,
                  T const * parameter3, T * residuals, T ** jacobians = nullptr) const;

 private:
  // Rotation from frame j to frame i.
  Eigen::Quaterniond i_qm_j_;
  // Translation from frame i to frame j, represented in i.
  Eigen::Vector3d i_tm_ij_;
  // The square root of the information matrix.
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

template <RotationUpdateMode mode>
class BetweenFactor : public ceres::SizedCostFunction<6, 4, 3, 4, 3> {
 public:
  BetweenFactor(Eigen::Quaterniond const & i_qm_j, Eigen::Vector3d const & i_tm_ij,
                Eigen::Matrix<double, 6, 6> const & sqrt_info);

  ~BetweenFactor() override;

  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override;

  static ceres::CostFunction * Create(Eigen::Quaterniond const & i_qm_j,
                                      Eigen::Vector3d const & i_tm_ij,
                                      Eigen::Matrix<double, 6, 6> const & sqrt_info,
                                      JacobianComputationMethod const & method);

 private:
  BetweenFunctor<mode> functor_;
};

#include "examples/ceres_solver/factors/between_factor.hpp"
