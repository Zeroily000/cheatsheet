#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * L(q) = [qw·I + [qv]x, qv]
 *        [       -qv^T, qw]
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionLeftMultiplicationMatrix(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * R(q) = [qw·I - [qv]x, qv]
 *        [       -qv^T, qw]
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4> QuaternionRightMultiplicationMatrix(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * dq(dw)·q0 ≈ (dw/2, 1)·q0
 *           = R(q0)·[dw^T/2, 1]^T
 *           = 0.5 * R(q0)·[I, 0]^T·dw + c
 *
 * J = 0.5 * R(q0)·[I, 0]^T
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> QuaternionLeftUpdateJacobian(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * q0·dq(dw) ≈ q0·(dw/2, 1)
 *           = L(q0)·[dw^T/2, 1]^T
 *           = 0.5 * L(q0)·[I, 0]^T·dw + c
 *
 * J = 0.5 * L(q0)·[I, 0]^T
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> QuaternionRightUpdateJacobian(
    Eigen::QuaternionBase<Derived> const & quaternion);

/**
 * @brief Computes the Jacobian of Log(q·q0^{-1}) w.r.t. q at q = q0.
 *
 * Let q1 = q·q0^{-1} = (qv1, qw1) and |Log(q1)| = |w1|, then:
 *
 * Log(q·q0^{-1}) = |w1|/sin(|w1|/2)·qv1
 *                = 2/sinc(|w1|/2)·qv1
 *                ≈ 2·qv1
 *                = 2·[I, 0]·q1
 *                = 2·[I, 0]·q·q0^{-1}
 *                = 2·[I, 0]·R(q0^{-1})·q
 *
 * J = 2·[I, 0]·R(q0^{-1})
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 4> QuaternionLeftUpdateJacobianInverse(
    Eigen::QuaternionBase<Derived> const & quaternion);

    /**
 * @brief Computes the Jacobian of Log(q(w)·q0^{-1}) w.r.t. q at q = q0.
 *
 * Let q1 = q0^{-1}·q = (qv1, qw1) and |Log(q1)| = |w1|, then:
 *
 * Log(q0^{-1}·q) = |w1|/sin(|w1|/2)·qv1
 *                = 2/sinc(|w1|/2)·qv1
 *                ≈ 2·qv1
 *                = 2·[I, 0]·q1
 *                = 2·[I, 0]·q0^{-1}·q
 *                = 2·[I, 0]·L(q0^{-1})·q
 *
 * J = 2·[I, 0]·L(q0^{-1})
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 4> QuaternionRightUpdateJacobianInverse(
    Eigen::QuaternionBase<Derived> const & quaternion);
