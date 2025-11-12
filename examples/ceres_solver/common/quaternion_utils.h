#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief q1 * q2 = L(q1)·q2
 *
 * L(q) = [qw·I + [qv]x, qv]
 *        [       -qv^T, qw]
 *
 * @param[in] quaternion A quaternion in the format (x, y, z, w)
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionLeftMultiplicationMatrix(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * @brief q1 * q2 = R(q2)·q1
 *
 * R(q) = [qw·I - [qv]x, qv]
 *        [       -qv^T, qw]
 *
 * @param[in] quaternion A quaternion in the format (x, y, z, w)
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionRightMultiplicationMatrix(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * @brief Derivative of q(w) * q0 w.r.t. w at w = 0
 *
 * q(w) * q0 = (sinc(|w|/2)/2·w, cos(|w|/2)) * q0
 *           ≈ (w/2, 1) * q0
 *           = R(q0)·[w^T/2, 1]^T
 *           = 0.5·R(q0)·[I, 0]^T·dw + c
 *
 * J = 0.5·R(q0)·[I, 0]^T
 *
 * @param[in] quaternion A unit quaternion in the format (x, y, z, w)
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> QuaternionLeftUpdateJacobian(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * @brief Derivative of q0 * q(w) w.r.t. w at w = 0
 *
 * q0 * q(w) = q0 * (sinc(|w|/2)/2·w, cos(|w|/2))
 *           ≈ q0 * (w/2, 1)
 *           = L(q0)·[w^T/2, 1]^T
 *           = 0.5·L(q0)·[I, 0]^T·dw + c
 *
 * J = 0.5·L(q0)·[I, 0]^T
 *
 * @param[in] quaternion A unit quaternion in the format (x, y, z, w)
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> QuaternionRightUpdateJacobian(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * @brief Computes the Jacobian of Log(q·q0^{-1}) w.r.t. q at q = q0.
 *
 * Let q1 = q * q0^{-1} = (qv1, qw1) and Log(q1) = w1, then:
 *
 * Log(q * q0^{-1}) = |w1|/sin(|w1|/2)·qv1
 *                  = 2/sinc(|w1|/2)·qv1
 *                  ≈ 2·qv1
 *                  = 2·[I, 0]·q1
 *                  = 2·[I, 0]·q·q0^{-1}
 *                  = 2·[I, 0]·R(q0^{-1})·q
 *
 * J = 2·[I, 0]·R(q0^{-1})
 *
 * @param[in] quaternion A unit quaternion in the format (x, y, z, w)
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 4> QuaternionLeftUpdateJacobianInverse(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * @brief Computes the Jacobian of Log(q0^{-1} * q) w.r.t. q at q = q0.
 *
 * Let q1 = q0^{-1} * q = (qv1, qw1) and Log(q1) = w1, then:
 *
 * Log(q0^{-1} * q) = |w1|/sin(|w1|/2)·qv1
 *                  = 2/sinc(|w1|/2)·qv1
 *                  ≈ 2·qv1
 *                  = 2·[I, 0]·q1
 *                  = 2·[I, 0]·q0^{-1}·q
 *                  = 2·[I, 0]·L(q0^{-1})·q
 *
 * J = 2·[I, 0]·L(q0^{-1})
 *
 * @param[in] quaternion A unit quaternion in the format (x, y, z, w)
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 4> QuaternionRightUpdateJacobianInverse(
    Eigen::QuaternionBase<Derived> const & quaternion);

#include "examples/ceres_solver/common/quaternion_utils.hpp"
