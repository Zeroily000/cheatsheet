#include "Eigen/Core"
#include "Eigen/Geometry"

/**
 * @brief This function computes the residual between two poses and the Jacobian w.r.t the rotation
 * and translation vector. The rotation uses the right update mode, i.e., q' = q * Exp(dw).
 * Specifically,
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
 * d/dwi e_w = d/dwi Log(r_R_j^{-1}·r_R_i·Exp(wi)·i_R_j)
 *           = d/dwi Log(r_R_j^{-1}·r_R_i·Exp(wi)·i_R_j)
 *           = d/dwi Log(r_R_j^{-1}·r_R_i·i_R_j·Exp(i_R_j^{-1}·wi))
 *           = d/dwi Log(e_R·Exp(i_R_j^{-1}·wi))
 *           = d/dwi Log(Exp(e_w + Jr^{-1}(e_w)·(i_R_j^{-1}·wi)))
 *           = Jr^{-1}(e_w)·i_R_j^{-1}
 *
 * d/dwi e_t = d/dwi (r_R_j^{-1}·r_R_i·Exp(wi)·i_t_ij
 *           = d/dwi (r_R_j^{-1}·r_R_i·(I + [wi]x)·i_t_ij
 *           = d/dwi (r_R_j^{-1}·r_R_i·[wi]x·i_t_ij)
 *           = d/dwi -(r_R_j^{-1}·r_R_i·[i_t_ij]x·wi)
 *           = -r_R_j^{-1}·r_R_i·[i_t_ij]x
 *
 * d/dti e_w = 0
 *
 * d/dti e_t = r_R_j^{-1}
 *
 * d/dwj e_w = d/dwj Log((r_R_j·Exp(wj))^{-1}·r_R_i·i_R_j)
 *           = d/dwj Log(Exp(-wj)·r_R_j^{-1}·r_R_i·i_R_j)
 *           = d/dwj Log(Exp(-wj)·e_R)
 *           = d/dwj Log(Exp(e_w - Jl^{-1}(e_w)·wj))
 *           = -Jl^{-1}(e_w)
 *
 * d/dwj e_t = d/dwj (r_R_j·Exp(wj))^{-1}·(r_R_i·i_t_ij + r_t_ri - r_t_rj)
 *           = d/dwj Exp(-wj)·r_R_j ^{-1}·(r_R_i·i_t_ij + r_t_ri - r_t_rj)
 *           = d/dwj (I - [w]x)·e_t
 *           = d/dwj -[w]x·e_t
 *           = [e_t]x
 *
 * d/dtj e_w = 0
 *
 * d/dtj e_t = -r_R_j^{-1}
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
                        Eigen::Quaternion<T> const & i_qm_j, Eigen::Matrix<T, 3, 1> const & i_tm_ij,
                        Eigen::Matrix<T, 6, 6> sqrt_info,
                        Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians);

/**
 * @brief This function computes the residual between two poses and the Jacobian w.r.t the rotation
 * and translation vector. The rotation uses the left update mode, i.e., q' = Exp(dw) * q.
 * Specifically,
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
 * d/dwi e_w = d/dwi Log(i_R_j·r_R_j^{-1}·Exp(wi)·r_R_i)
 *           = d/dwi Log(i_R_j·r_R_j^{-1}·r_R_i·Exp(r_R_i^{-1}·wi)
 *           = d/dwi Log(e_R·Exp(r_R_i^{-1}·wi))
 *           = d/dwi Log(Exp(e_w + Jr^{-1}(e_w)·(r_R_i^{-1}·wi))
 *           = Jr^{-1}(e_w)·r_R_i^{-1}
 *
 * d/dwi e_t = 0
 *
 * d/dti e_w = 0
 *
 * d/dti e_t = i_R_j·r_R_j^{-1}
 *
 * d/dwj e_w = d/dwj Log(i_R_j·(Exp(wj)·r_R_j)^{-1}·r_R_i)
 *           = d/dwj Log(i_R_j·r_R_j^{-1}·Exp(-wj)·r_R_i)
 *           = d/dwj Log(i_R_j·r_R_j^{-1}·r_R_i·Exp(-r_R_i^{-1}·wj))
 *           = d/dwj Log(e_R·Exp(-r_R_i^{-1}·wj))
 *           = d/dwj Log(Exp(e_w - Jr^{-1}(e_w)·r_R_i^{-1}·wj))
 *           = -Jr^{-1}(e_w)·r_R_i^{-1}
 *
 * d/dwj e_t = d/dwj i_R_j·(Exp(wj)·r_R_j)^{-1}·(r_t_ri - r_t_rj)
 *           = d/dwj i_R_j·r_R_j^{-1}·Exp(-wj)·(r_t_ri - r_t_rj)
 *           = d/dwj i_R_j·r_R_j^{-1}·(I - [wj]x)·(r_t_ri - r_t_rj)
 *           = d/dwj i_R_j·r_R_j^{-1}·[-wj]x·(r_t_ri - r_t_rj)
 *           = d/dwj i_R_j·r_R_j^{-1}·[r_t_ri - r_t_rj]x·wj
 *           = i_R_j·r_R_j^{-1}·[r_t_ri - r_t_rj]x
 *
 * d/dtj e_w = 0
 *
 * d/dtj e_t = -i_R_j·r_R_j^{-1}
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
bool EvaluateLeftOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_i,
                       Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ri,
                       Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_j,
                       Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rj,
                       Eigen::Quaternion<T> const & i_qm_j, Eigen::Matrix<T, 3, 1> const & i_tm_ij,
                       Eigen::Matrix<T, 6, 6> sqrt_info,
                       Eigen::Map<Eigen::Matrix<T, 6, 1>> & whitened_error, T ** jacobians);
