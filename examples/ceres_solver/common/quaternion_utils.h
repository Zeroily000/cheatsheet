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
