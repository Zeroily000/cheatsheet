#include "examples/ceres_solver/factors/prior_factor.h"

#include <gtest/gtest.h>

#include <sophus/so3.hpp>

#include "examples/ceres_solver/common/rotation_manifold.hpp"

namespace {
template <RotationUpdateMode mode>
void TestPriorFactor() {
  static double constexpr kTolerance{1e-4};
  std::srand(0);
  Eigen::Quaterniond const r_qm_i{Sophus::SO3d::exp(Eigen::Vector3d::Random()).unit_quaternion()};
  Eigen::Vector3d const r_tm_ri{Eigen::Vector3d::Random()};
  Eigen::Matrix<double, 6, 6> sqrt_info{Eigen::Matrix<double, 6, 6>::Identity()};
  PriorFactor<mode> const factor{r_qm_i, r_tm_ri, sqrt_info};

  Eigen::Quaterniond const r_qe_i{
      r_qm_i * Sophus::SO3d::exp(Eigen::Vector3d::Random() * 1e-3).unit_quaternion()};
  Eigen::Vector3d const r_te_ri{r_tm_ri + Eigen::Vector3d::Random() * 1e-2};

  Eigen::Matrix<double, 6, 1> residuals0;
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dq_analytic;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dt_analytic;

  {
    std::array<double const *, 2> const parameters{r_qe_i.coeffs().data(), r_te_ri.data()};
    std::array<double *, 2> jacobians{dr_dq_analytic.data(), dr_dt_analytic.data()};
    factor.Evaluate(parameters.data(), residuals0.data(), jacobians.data());
  }
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dq_dw;
  RotationManifold<mode> rotation_manifold;
  rotation_manifold.PlusJacobian(r_qe_i.coeffs().data(), dq_dw.data());
  Eigen::Matrix<double, 6, 3> const dr_dw_analytic{dr_dq_analytic * dq_dw};

  Eigen::Vector3d const dw{Eigen::Vector3d::Random() * 1e-9};
  Eigen::Vector3d const dwx{Eigen::Vector3d::UnitX() * dw.x()};
  Eigen::Vector3d const dwy{Eigen::Vector3d::UnitY() * dw.y()};
  Eigen::Vector3d const dwz{Eigen::Vector3d::UnitZ() * dw.z()};

  Eigen::Vector3d const dt{Eigen::Vector3d::Random() * 1e-9};
  Eigen::Vector3d const dtx{Eigen::Vector3d::UnitX() * dt.x()};
  Eigen::Vector3d const dty{Eigen::Vector3d::UnitY() * dt.y()};
  Eigen::Vector3d const dtz{Eigen::Vector3d::UnitZ() * dt.z()};

  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dw_numeric;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dt_numeric;

  {
    Eigen::Quaterniond qadqx;
    rotation_manifold.Plus(r_qe_i.coeffs().data(), dwx.data(), qadqx.coeffs().data());
    std::array<double const *, 2> const parameters{qadqx.coeffs().data(), r_te_ri.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dw_numeric.col(0) = (residuals1 - residuals0) / dw.x();
  }
  {
    Eigen::Quaterniond qadqy;
    rotation_manifold.Plus(r_qe_i.coeffs().data(), dwy.data(), qadqy.coeffs().data());
    std::array<double const *, 2> const parameters{qadqy.coeffs().data(), r_te_ri.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dw_numeric.col(1) = (residuals1 - residuals0) / dw.y();
  }
  {
    Eigen::Quaterniond qadqz;
    rotation_manifold.Plus(r_qe_i.coeffs().data(), dwz.data(), qadqz.coeffs().data());
    std::array<double const *, 2> const parameters{qadqz.coeffs().data(), r_te_ri.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dw_numeric.col(2) = (residuals1 - residuals0) / dw.z();
  }
  EXPECT_TRUE(dr_dw_analytic.isApprox(dr_dw_numeric, kTolerance));
  // std::cout << dr_dw_analytic << std::endl;
  // std::cout << "-----" << std::endl;
  // std::cout << dr_dw_numeric << std::endl;

  {
    Eigen::Vector3d const tadtx{r_te_ri + dtx};
    std::array<double const *, 2> const parameters{r_qe_i.coeffs().data(), tadtx.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dt_numeric.col(0) = (residuals1 - residuals0) / dt.x();
  }
  {
    Eigen::Vector3d const tadty{r_te_ri + dty};
    std::array<double const *, 2> const parameters{r_qe_i.coeffs().data(), tadty.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dt_numeric.col(1) = (residuals1 - residuals0) / dt.y();
  }
  {
    Eigen::Vector3d const tadtz{r_te_ri + dtz};
    std::array<double const *, 2> const parameters{r_qe_i.coeffs().data(), tadtz.data()};
    Eigen::Matrix<double, 6, 1> residuals1;
    factor.Evaluate(parameters.data(), residuals1.data(), nullptr);
    dr_dt_numeric.col(2) = (residuals1 - residuals0) / dt.z();
  }
  EXPECT_TRUE(dr_dt_analytic.isApprox(dr_dt_numeric, kTolerance));
  // std::cout << dr_dt_analytic << std::endl;
  // std::cout << "-----" << std::endl;
  // std::cout << dr_dt_numeric << std::endl;
}

TEST(PriorFactorTest, TestRightUpdateMode) { TestPriorFactor<RotationUpdateMode::kRight>(); }

// TEST(PriorFactorTest, TestLeftUpdateMode) { TestPriorFactor<RotationUpdateMode::kLeft>(); }

}  // namespace
