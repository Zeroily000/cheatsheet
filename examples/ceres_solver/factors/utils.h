#include "Eigen/Core"
#include "Eigen/Geometry"

/**
 * @brief This function computes the residual between two poses and the Jacobian w.r.t the rotation
 * and translation vector. The rotation uses the right update mode, i.e., q' = q * Exp(dw).
 * Specifically,
 *
 * a_T_b·e = z
 * r_T_a^{-1}·r_T_b·e = z
 * e = r_T_b^{-1}·r_T_a·z
 *   = [r_R_b^{-1}, -r_R_b^{-1}·r_t_rb]·[r_R_a, r_t_ra]·[a_R_b, a_t_ab]
 *     [         0,                  1] [   0,       1] [    0,      1]
 *   = [r_R_b^{-1}·r_R_a·a_R_b, r_R_b^{-1}·r_R_a·a_t_ab + r_R_b^{-1}·(r_t_ra - r_t_rb)]
 *     [                     0,                                                      1]
 *
 * e_w = Log(r_R_b^{-1}·r_R_a·a_R_b)
 * e_t = r_R_b^{-1}·(r_R_a·a_t_ab + r_t_ra - r_t_rb)
 *
 * d/dwa e_w = d/dwa Log(r_R_b^{-1}·r_R_a·Exp(wa)·a_R_b)
 *           = d/dwa Log(r_R_b^{-1}·r_R_a·Exp(wa)·a_R_b)
 *           = d/dwa Log(r_R_b^{-1}·r_R_a·a_R_b·Exp(a_R_b^{-1}·wa))
 *           = d/dwa Log(e_R·Exp(a_R_b^{-1}·wa))
 *           = d/dwa Log(Exp(e_w + Jr^{-1}(e_w)·(a_R_b^{-1}·wa)))
 *           = Jr^{-1}(e_w)·a_R_b^{-1}
 *
 * d/dwa e_t = d/dwa (r_R_b^{-1}·r_R_a·Exp(wa)·a_t_ab
 *           = d/dwa (r_R_b^{-1}·r_R_a·(I + [wa]x)·a_t_ab
 *           = d/dwa (r_R_b^{-1}·r_R_a·[wa]x·a_t_ab)
 *           = d/dwa -(r_R_b^{-1}·r_R_a·[a_t_ab]x·wa)
 *           = -r_R_b^{-1}·r_R_a·[a_t_ab]x
 *
 * d/dta e_w = 0
 *
 * d/dta e_t = r_R_b^{-1}
 *
 * d/dwb e_w = d/dwb Log((r_R_b·Exp(wb))^{-1}·r_R_a·a_R_b)
 *           = d/dwb Log(Exp(-wb)·r_R_b^{-1}·r_R_a·a_R_b)
 *           = d/dwb Log(Exp(-wb)·e_R)
 *           = d/dwb Log(Exp(e_w - Jl^{-1}(e_w)·wb))
 *           = -Jl^{-1}(e_w)
 *
 * d/dwb e_t = d/dwb (r_R_b·Exp(wb))^{-1}·(r_R_a·a_t_ab + r_t_ra - r_t_rb)
 *           = d/dwb Exp(-wb)·r_R_b ^{-1}·(r_R_a·a_t_ab + r_t_ra - r_t_rb)
 *           = d/dwb (I - [w]x)·e_t
 *           = d/dwb -[w]x·e_t
 *           = [e_t]x
 *
 * d/dtb e_w = 0
 *
 * d/dtb e_t = -r_R_b^{-1}
 *
 * @param[in] r_qe_i The estimated rotation from frame i to the reference frame.
 * @param[in] r_te_ri The estimated translation from the reference frame to frame i represented in
 * the reference frame.
 * @param[in] r_qe_j The estimated rotation from frame j to the reference frame.
 * @param[in] r_te_rj The estimated translation from the reference frame to frame j represented in
 * the reference frame.
 * @param[in] i_qm_j The measured rotation from frame j to frame i;
 * @param[in] i_tm_ij The measured translation from the frame j to frame i represented in frame i.
 * @param[in] sqrt_info The square root of the information matrix.
 * @param[out] whitened_error The residual.
 * @param[out] jacobians The jacobians of the residual w.r.t each parameter.
 */
template <typename T>
bool EvaluateRightOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
                        Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
                        Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
                        Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj,
                        Eigen::Quaternion<T> const & i_qm_j, Eigen::Matrix<T, 3, 1> const & i_tm_oj,
                        Eigen::Matrix<T, 6, 6> sqrt_info,
                        Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians);

/**
 * @brief This function computes the residual between two poses and the Jacobian w.r.t the rotation
 * and translation vector. The rotation uses the left update mode, i.e., q' = Exp(dw) * q.
 * Specifically,
 *
 * a_T_b·e = z
 * e·r_T_a^{-1}·r_T_b = z
 *
 * e = z·r_T_b^{-1}·r_T_a
 *   = [a_R_b, a_t_ab]·[r_R_b^{-1}, -r_R_b^{-1}·r_t_rb]·[r_R_a, r_t_ra]
 *     [    0,      1] [         0,                  1] [   0,      1]
 *   = [a_R_b·r_R_b^{-1}·r_R_a, a_R_b·r_R_b^{-1}·(r_t_ra - r_t_rb) + a_t_ab]
 *     [                     0,                                           1]
 *
 * e_w = Log(a_R_b·r_R_b^{-1}·r_R_a)
 * e_t = a_R_b·r_R_b^{-1}·(r_t_ra - r_t_rb) + a_t_ab
 *
 * d/dwa e_w = d/dwa Log(a_R_b·r_R_b^{-1}·Exp(wa)·r_R_a)
 *           = d/dwa Log(a_R_b·r_R_b^{-1}·r_R_a·Exp(r_R_a^{-1}·wa)
 *           = d/dwa Log(e_R·Exp(r_R_a^{-1}·wa))
 *           = d/dwa Log(Exp(e_w + Jr^{-1}(e_w)·(r_R_a^{-1}·wa))
 *           = Jr^{-1}(e_w)·r_R_a^{-1}
 *
 * d/dw e_t = 0
 *
 * d/dta e_w = 0
 *
 * d/dta e_t = a_R_b·r_R_b^{-1}
 *
 * d/dwb e_w = d/dwb Log(a_R_b·(Exp(wb)·r_R_b)^{-1}·r_R_a)
 *           = d/dwb Log(a_R_b·r_R_b^{-1}·Exp(-wb)·r_R_a)
 *           = d/dwb Log(a_R_b·r_R_b^{-1}·r_R_a·Exp(-r_R_a^{-1}·wb))
 *           = d/dwb Log(e_R·Exp(-r_R_a^{-1}·wb))
 *           = d/dwb Log(Exp(e_w - Jr^{-1}(e_w)·r_R_a^{-1}·wb))
 *           = -Jr^{-1}(e_w)·r_R_a^{-1}
 *
 * d/dwb e_t = d/dwb a_R_b·(Exp(wb)·r_R_b)^{-1}·(r_t_ra - r_t_rb)
 *           = d/dwb a_R_b·r_R_b^{-1}·Exp(-wb)·(r_t_ra - r_t_rb)
 *           = d/dwb a_R_b·r_R_b^{-1}·(I - [wb]x)·(r_t_ra - r_t_rb)
 *           = d/dwb a_R_b·r_R_b^{-1}·[-wb]x·(r_t_ra - r_t_rb)
 *           = d/dwb a_R_b·r_R_b^{-1}·[r_t_ra - r_t_rb]x·wb
 *           = a_R_b·r_R_b^{-1}·[r_t_ra - r_t_rb]x
 *
 * d/dt e_w = 0
 *
 * d/dt e_t = -a_R_b·r_R_b^{-1}
 *
 * @param[in] r_qe_i The estimated rotation from frame i to the reference frame.
 * @param[in] r_te_ri The estimated translation from the reference frame to frame i represented in
 * the reference frame.
 * @param[in] r_qe_j The estimated rotation from frame j to the reference frame.
 * @param[in] r_te_rj The estimated translation from the reference frame to frame j represented in
 * the reference frame.
 * @param[in] i_qm_j The measured rotation from frame j to frame i;
 * @param[in] i_tm_ij The measured translation from the frame j to frame i represented in frame i.
 * @param[in] sqrt_info The square root of the information matrix.
 * @param[out] whitened_error The residual.
 * @param[out] jacobians The jacobians of the residual w.r.t each parameter.
 */
template <typename T>
static bool EvaluateLeftOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
                              Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj,
                              Eigen::Quaternion<T> const & i_qm_j,
                              Eigen::Matrix<T, 3, 1> const & i_tm_ij,
                              Eigen::Matrix<T, 6, 6> sqrt_info,
                              Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians);
