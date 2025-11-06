#include "examples/ceres_solver/factors/between_factor.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/rotation_manifold.hpp"

namespace {

// struct AutoDiffBetweenFactor {
//   AutoDiffBetweenFactor(Eigen::Quaterniond const & a_q_b, Eigen::Vector3d const & a_t_ab,
//                         Eigen::Matrix<double, 6, 6> const & sqrt_info)
//       : a_q_b_{std::move(a_q_b)}, a_t_ab_{std::move(a_t_ab)}, sqrt_info_{std::move(sqrt_info)} {}

//   template <typename T>
//   bool operator()(T const * const qa, T const * const ta, T const * const qb, T const * const tb,
//                   T * const residuals) const {
//     Eigen::Map<Eigen::Quaternion<T> const> const r_qe_a{qa};
//     Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_ra{ta};
//     Eigen::Map<Eigen::Quaternion<T> const> const r_qe_b{qb};
//     Eigen::Map<Eigen::Matrix<T, 3, 1> const> const r_te_rb{tb};
//     Eigen::Quaternion<T> const a_qm_b{a_q_b_.cast<T>()};
//     Eigen::Matrix<T, 3, 1> const a_tm_ab{a_t_ab_.cast<T>()};
//     Eigen::Matrix<T, 6, 6> const sqrt_info{sqrt_info_.cast<T>()};
//     Eigen::Map<Eigen::Matrix<T, 6, 1>> whitened_error{residuals};
//     T ** jacobians{nullptr};
//     return BetweenFactor<RotationUpdateMode::kLeft>::Evaluate(
//         r_qe_a, r_te_ra, r_qe_b, r_te_rb, a_qm_b, a_tm_ab, sqrt_info, whitened_error, jacobians);
//   }

//   static auto * Create(Eigen::Quaterniond const & a_q_b, Eigen::Vector3d const & a_t_ab,
//                        Eigen::Matrix<double, 6, 6> const & sqrt_info) {
//     return new ceres::AutoDiffCostFunction<AutoDiffBetweenFactor, 6, 4, 3, 4, 3>(
//         new AutoDiffBetweenFactor{a_q_b, a_t_ab, sqrt_info});
//   }

//  private:
//   // Rotation from frame b to frame a
//   Eigen::Quaterniond a_q_b_;
//   // Translation from frame a to frame b, represented in a.
//   Eigen::Vector3d a_t_ab_;

//   Eigen::Matrix<double, 6, 6> sqrt_info_;
// };
template <RotationUpdateMode mode>
void TestBetweenFactor() {
  Eigen::Quaterniond const qa{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d const ta{Eigen::Vector3d::Random()};
  Eigen::Quaterniond const qb{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d const tb{Eigen::Vector3d::Random()};

  Eigen::Quaterniond const qz{qa.inverse() * qb};
  Eigen::Vector3d const tz{qa.inverse() * (tb - ta)};
  Eigen::Matrix<double, 6, 6> sqrt_info{Eigen::Matrix<double, 6, 6>::Identity()};
  BetweenFactor<mode> const factor{qz, tz, sqrt_info};

  Eigen::Matrix<double, 6, 1> residuals0;
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqa_analytic;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_analytic;
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqb_analytic;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_analytic;

  {
    std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
                                                   qb.coeffs().data(), tb.data()};
    std::array<double *, 4> jacobians{dr_dqa_analytic.data(), dr_dta_analytic.data(),
                                      dr_dqb_analytic.data(), dr_dtb_analytic.data()};
    factor.Evaluate(parameters.data(), residuals0.data(), jacobians.data());
  }
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqa_dwa;
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqb_dwb;
  RotationManifold<mode> rotation_manifold;
  rotation_manifold.PlusJacobian(qa.coeffs().data(), dqa_dwa.data());
  rotation_manifold.PlusJacobian(qb.coeffs().data(), dqb_dwb.data());
  Eigen::Matrix<double, 6, 3> const dr_dwa_analytic = dr_dqa_analytic * dqa_dwa;
  Eigen::Matrix<double, 6, 3> const dr_dwb_analytic = dr_dqb_analytic * dqb_dwb;

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
    rotation_manifold.Plus(qa.coeffs().data(), dwx.data(), qadqx.coeffs().data());
    std::array<double const *, 4> const parameters{qadqx.coeffs().data(), ta.data(),
                                                   qb.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwa_numeric.col(0) = (residuals1 - residuals0) / dw.x();
  }
  {
    Eigen::Quaterniond qadqy;
    rotation_manifold.Plus(qa.coeffs().data(), dwy.data(), qadqy.coeffs().data());
    std::array<double const *, 4> const parameters{qadqy.coeffs().data(), ta.data(),
                                                   qb.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwa_numeric.col(1) = (residuals1 - residuals0) / dw.y();
  }
  {
    Eigen::Quaterniond qadqz;
    rotation_manifold.Plus(qa.coeffs().data(), dwz.data(), qadqz.coeffs().data());
    std::array<double const *, 4> const parameters{qadqz.coeffs().data(), ta.data(),
                                                   qb.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwa_numeric.col(2) = (residuals1 - residuals0) / dw.z();
  }
  EXPECT_TRUE(dr_dwa_analytic.isApprox(dr_dwa_numeric, 1e-4));

  {
    Eigen::Quaterniond qbdqx;
    rotation_manifold.Plus(qb.coeffs().data(), dwx.data(), qbdqx.coeffs().data());
    std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
                                                   qbdqx.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwb_numeric.col(0) = (residuals1 - residuals0) / dw.x();
  }
  {
    Eigen::Quaterniond qbdqy;
    rotation_manifold.Plus(qb.coeffs().data(), dwy.data(), qbdqy.coeffs().data());
    std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
                                                   qbdqy.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwb_numeric.col(1) = (residuals1 - residuals0) / dw.y();
  }
  {
    Eigen::Quaterniond qbdqz;
    rotation_manifold.Plus(qb.coeffs().data(), dwz.data(), qbdqz.coeffs().data());
    std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
                                                   qbdqz.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dwb_numeric.col(2) = (residuals1 - residuals0) / dw.z();
  }
  EXPECT_TRUE(dr_dwb_analytic.isApprox(dr_dwb_numeric, 1e-4));

  {
    Eigen::Vector3d const tadtx{ta + dtx};
    std::array<double const *, 4> const parameters{qa.coeffs().data(), tadtx.data(),
                                                   qb.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dta_numeric.col(0) = (residuals1 - residuals0) / dt.x();
  }
  {
    Eigen::Vector3d const tadty{ta + dty};
    std::array<double const *, 4> const parameters{qa.coeffs().data(), tadty.data(),
                                                   qb.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dta_numeric.col(1) = (residuals1 - residuals0) / dt.y();
  }
  {
    Eigen::Vector3d const tadtz{ta + dtz};
    std::array<double const *, 4> const parameters{qa.coeffs().data(), tadtz.data(),
                                                   qb.coeffs().data(), tb.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dta_numeric.col(2) = (residuals1 - residuals0) / dt.z();
  }
  EXPECT_TRUE(dr_dta_analytic.isApprox(dr_dta_numeric, 1e-6));

  {
    Eigen::Vector3d const tbdtx{tb + dtx};
    std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
                                                   qb.coeffs().data(), tbdtx.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dtb_numeric.col(0) = (residuals1 - residuals0) / dt.x();
  }
  {
    Eigen::Vector3d const tbdty{tb + dty};
    std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
                                                   qb.coeffs().data(), tbdty.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dtb_numeric.col(1) = (residuals1 - residuals0) / dt.y();
  }
  {
    Eigen::Vector3d const tbdtz{tb + dtz};
    std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(),
                                                   qb.coeffs().data(), tbdtz.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dtb_numeric.col(2) = (residuals1 - residuals0) / dt.z();
  }
  EXPECT_TRUE(dr_dtb_analytic.isApprox(dr_dtb_numeric, 1e-6));
}

}  // namespace

TEST(RotationManifoldTest, TestRightPerturbation) {
  TestBetweenFactor<RotationUpdateMode::kRight>();
  TestBetweenFactor<RotationUpdateMode::kLeft>();
}

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
