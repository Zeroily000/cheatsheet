#include "examples/ceres_solver/factors/between_factor.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/rotation_manifold.hpp"

namespace {

template <RotationUpdateMode mode>
void TestBetweenFactor() {
  Eigen::Quaterniond const r_qm_i{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d const r_tm_ri{Eigen::Vector3d::Random()};
  Eigen::Quaterniond const r_qm_j{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d const r_tm_rj{Eigen::Vector3d::Random()};

  Eigen::Quaterniond const i_qm_j{r_qm_i.inverse() * r_qm_j};
  Eigen::Vector3d const i_tm_ij{r_qm_i.inverse() * (r_tm_rj - r_tm_ri)};
  Eigen::Matrix<double, 6, 6> sqrt_info{Eigen::Matrix<double, 6, 6>::Identity()};
  BetweenFactor<mode> const factor{i_qm_j, i_tm_ij, sqrt_info};

  Eigen::Quaterniond const r_qe_i{
      r_qm_i * Sophus::SO3d::exp(Eigen::Vector3d::Random() * 1e-3).unit_quaternion()};
  Eigen::Vector3d const r_te_ri{r_tm_ri + Eigen::Vector3d::Random() * 1e-2};
  Eigen::Quaterniond const r_qe_j{
      r_qm_j * Sophus::SO3d::exp(Eigen::Vector3d::Random() * 1e-3).unit_quaternion()};
  Eigen::Vector3d const r_te_rj{r_tm_rj + Eigen::Vector3d::Random() * 1e-2};

  Eigen::Matrix<double, 6, 1> residuals0;
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqa_analytic;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_analytic;
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqb_analytic;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_analytic;

  {
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), r_te_ri.data(),
                                                   r_qe_j.coeffs().data(), r_te_rj.data()};
    std::array<double *, 4> jacobians{dr_dqa_analytic.data(), dr_dta_analytic.data(),
                                      dr_dqb_analytic.data(), dr_dtb_analytic.data()};
    factor.Evaluate(parameters.data(), residuals0.data(), jacobians.data());
  }
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqa_dwa;
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqb_dwb;
  RotationManifold<mode> rotation_manifold;
  rotation_manifold.PlusJacobian(r_qe_i.coeffs().data(), dqa_dwa.data());
  rotation_manifold.PlusJacobian(r_qe_j.coeffs().data(), dqb_dwb.data());
  Eigen::Matrix<double, 6, 3> const dr_dwa_analytic{dr_dqa_analytic * dqa_dwa};
  Eigen::Matrix<double, 6, 3> const dr_dwb_analytic{dr_dqb_analytic * dqb_dwb};

  Eigen::Vector3d const dw{Eigen::Vector3d::Random() * 1e-9};
  Eigen::Vector3d const dwx{Eigen::Vector3d::UnitX() * dw.x()};
  Eigen::Vector3d const dwy{Eigen::Vector3d::UnitY() * dw.y()};
  Eigen::Vector3d const dwz{Eigen::Vector3d::UnitZ() * dw.z()};

  Eigen::Vector3d const dt{Eigen::Vector3d::Random() * 1e-9};
  Eigen::Vector3d const dtx{Eigen::Vector3d::UnitX() * dt.x()};
  Eigen::Vector3d const dty{Eigen::Vector3d::UnitY() * dt.y()};
  Eigen::Vector3d const dtz{Eigen::Vector3d::UnitZ() * dt.z()};

  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dwa_numeric;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dwb_numeric;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_numeric;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_numeric;

  {
    // Eigen::Quaterniond const qadqx{qa * dqx};
    Eigen::Quaterniond qadqx;
    rotation_manifold.Plus(r_qe_i.coeffs().data(), dwx.data(), qadqx.coeffs().data());
    std::array<double const *, 4> const parameters{qadqx.coeffs().data(), r_te_ri.data(),
                                                   r_qe_j.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwa_numeric.col(0) = (residuals1 - residuals0) / dw.x();
  }
  {
    Eigen::Quaterniond qadqy;
    rotation_manifold.Plus(r_qe_i.coeffs().data(), dwy.data(), qadqy.coeffs().data());
    std::array<double const *, 4> const parameters{qadqy.coeffs().data(), r_te_ri.data(),
                                                   r_qe_j.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwa_numeric.col(1) = (residuals1 - residuals0) / dw.y();
  }
  {
    Eigen::Quaterniond qadqz;
    rotation_manifold.Plus(r_qe_i.coeffs().data(), dwz.data(), qadqz.coeffs().data());
    std::array<double const *, 4> const parameters{qadqz.coeffs().data(), r_te_ri.data(),
                                                   r_qe_j.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwa_numeric.col(2) = (residuals1 - residuals0) / dw.z();
  }
  EXPECT_TRUE(dr_dwa_analytic.isApprox(dr_dwa_numeric, 1e-4));

  {
    Eigen::Quaterniond qbdqx;
    rotation_manifold.Plus(r_qe_j.coeffs().data(), dwx.data(), qbdqx.coeffs().data());
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), r_te_ri.data(),
                                                   qbdqx.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwb_numeric.col(0) = (residuals1 - residuals0) / dw.x();
  }
  {
    Eigen::Quaterniond qbdqy;
    rotation_manifold.Plus(r_qe_j.coeffs().data(), dwy.data(), qbdqy.coeffs().data());
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), r_te_ri.data(),
                                                   qbdqy.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwb_numeric.col(1) = (residuals1 - residuals0) / dw.y();
  }
  {
    Eigen::Quaterniond qbdqz;
    rotation_manifold.Plus(r_qe_j.coeffs().data(), dwz.data(), qbdqz.coeffs().data());
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), r_te_ri.data(),
                                                   qbdqz.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwb_numeric.col(2) = (residuals1 - residuals0) / dw.z();
  }
  EXPECT_TRUE(dr_dwb_analytic.isApprox(dr_dwb_numeric, 1e-4));

  {
    Eigen::Vector3d const tadtx{r_te_ri + dtx};
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), tadtx.data(),
                                                   r_qe_j.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dta_numeric.col(0) = (residuals1 - residuals0) / dt.x();
  }
  {
    Eigen::Vector3d const tadty{r_te_ri + dty};
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), tadty.data(),
                                                   r_qe_j.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dta_numeric.col(1) = (residuals1 - residuals0) / dt.y();
  }
  {
    Eigen::Vector3d const tadtz{r_te_ri + dtz};
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), tadtz.data(),
                                                   r_qe_j.coeffs().data(), r_te_rj.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dta_numeric.col(2) = (residuals1 - residuals0) / dt.z();
  }
  EXPECT_TRUE(dr_dta_analytic.isApprox(dr_dta_numeric, 1e-4));

  {
    Eigen::Vector3d const tbdtx{r_te_rj + dtx};
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), r_te_ri.data(),
                                                   r_qe_j.coeffs().data(), tbdtx.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dtb_numeric.col(0) = (residuals1 - residuals0) / dt.x();
  }
  {
    Eigen::Vector3d const tbdty{r_te_rj + dty};
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), r_te_ri.data(),
                                                   r_qe_j.coeffs().data(), tbdty.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dtb_numeric.col(1) = (residuals1 - residuals0) / dt.y();
  }
  {
    Eigen::Vector3d const tbdtz{r_te_rj + dtz};
    std::array<double const *, 4> const parameters{r_qe_i.coeffs().data(), r_te_ri.data(),
                                                   r_qe_j.coeffs().data(), tbdtz.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dtb_numeric.col(2) = (residuals1 - residuals0) / dt.z();
  }
  EXPECT_TRUE(dr_dtb_analytic.isApprox(dr_dtb_numeric, 1e-4));
}

}  // namespace

TEST(RotationManifoldTest, TestRightUpdateMode) { TestBetweenFactor<RotationUpdateMode::kRight>(); }

TEST(RotationManifoldTest, TestLeftUpdateMode) { TestBetweenFactor<RotationUpdateMode::kLeft>(); }

// TEST(RotationManifoldTest, TestLeftPerturbation) {
//   // RotationManifold rotation_manifold{RotationManifold::Mode::kLeftPerturbation};
//   Eigen::Quaterniond const qa{Eigen::Quaterniond::UnitRandom()};
//   Eigen::Vector3d const ta{Eigen::Vector3d::Random()};
//   Eigen::Quaterniond const qb{Eigen::Quaterniond::UnitRandom()};
//   Eigen::Vector3d const tb{Eigen::Vector3d::Random()};
//   std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
//   qb.coeffs().data(),
//                                                  tb.data()};

//   Eigen::Quaterniond const qz{qa.inverse() * qb};
//   Eigen::Vector3d const tz{qa.inverse() * (tb - ta)};
//   Eigen::Matrix<double, 6, 6> sqrt_info{Eigen::Matrix<double, 6, 6>::Identity()};
//   // BetweenFactor<RotationUpdateMode::kLeft> const factor{qz, tz, sqrt_info};
//   ceres::CostFunction const * const analytic_factor{
//       BetweenFactor<RotationUpdateMode::kLeft>::Create(qz, tz, sqrt_info,
//                                                        JacobianComputationMethod::kAnalytic)};

//   Eigen::Matrix<double, 6, 1> residuals;
//   Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dwa_analytic;
//   Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_analytic;
//   Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dwb_analytic;
//   Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_analytic;

//   {
//     std::array<double *, 4> jacobians{dr_dwa_analytic.data(), dr_dta_analytic.data(),
//                                       dr_dwb_analytic.data(), dr_dtb_analytic.data()};
//     analytic_factor->Evaluate(parameters.data(), residuals.data(), jacobians.data());
//   }

//   Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqa;
//   Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqb;
//   Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqa_dwa;
//   Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqb_dwb;
//   auto const * manifold{new ceres::EigenQuaternionManifold};
//   manifold->PlusJacobian(qa.coeffs().data(), dqa_dwa.data());
//   manifold->PlusJacobian(qb.coeffs().data(), dqb_dwb.data());

//   // auto const * autodiff_factor = AutoDiffBetweenFactor::Create(qz, tz, sqrt_info);
//   ceres::CostFunction const * const autodiff_factor{
//       BetweenFactor<RotationUpdateMode::kLeft>::Create(qz, tz, sqrt_info,
//                                                        JacobianComputationMethod::kAutomatic)};

//   Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dwa_autodiff;
//   Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dwb_autodiff;
//   Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_autodiff;
//   Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_autodiff;
//   {
//     std::array<double *, 4> jacobians{dr_dqa.data(), dr_dta_autodiff.data(), dr_dqb.data(),
//                                       dr_dtb_autodiff.data()};
//     autodiff_factor->Evaluate(parameters.data(), residuals.data(), jacobians.data());
//     dr_dwa_autodiff = dr_dqa * dqa_dwa * .5;
//     dr_dwb_autodiff = dr_dqb * dqb_dwb * .5;
//   }
//   EXPECT_TRUE(dr_dwa_analytic.leftCols(3).isApprox(dr_dwa_autodiff));
//   EXPECT_TRUE(dr_dwb_analytic.leftCols(3).isApprox(dr_dwb_autodiff));
//   EXPECT_TRUE(dr_dta_analytic.isApprox(dr_dta_autodiff));
//   EXPECT_TRUE(dr_dtb_analytic.isApprox(dr_dtb_autodiff));

//   delete manifold;
//   delete analytic_factor;
//   delete autodiff_factor;
// }
