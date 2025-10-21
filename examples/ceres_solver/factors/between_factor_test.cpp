#include "examples/ceres_solver/factors/between_factor.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <sophus/so3.hpp>

namespace {

struct AutoDiffBetweenFactor {
  AutoDiffBetweenFactor(Eigen::Quaterniond a_R_b, Eigen::Vector3d a_t_b,
                        Eigen::Matrix<double, 6, 6> sqrt_info)
      : a_R_b_{std::move(a_R_b)}, a_t_b_{std::move(a_t_b)}, sqrt_info_{std::move(sqrt_info)} {}

  template <typename T>
  bool operator()(T const * const qa, T const * const ta, T const * const qb, T const * const tb,
                  T * const residuals) const {
    Eigen::Map<Eigen::Quaternion<T> const> const ref_R_a{qa};
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const ref_t_a{ta};

    Eigen::Map<Eigen::Quaternion<T> const> const ref_R_b{qb};
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const ref_t_b{tb};

    Eigen::Quaternion<T> const b_R_ref{ref_R_b.inverse()};
    Eigen::Matrix<T, 6, 1> error;
    error.template head<3>() = Sophus::SO3<T>{b_R_ref * ref_R_a * a_R_b_.cast<T>()}.log();
    error.template tail<3>() = b_R_ref * (ref_R_a * a_t_b_.cast<T>() + ref_t_a - ref_t_b);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> res{residuals};
    res = sqrt_info_.cast<T>() * error;

    return true;
  }

  static ceres::CostFunction * Create(Eigen::Quaterniond const & a_R_b,
                                      Eigen::Vector3d const & a_t_b,
                                      Eigen::Matrix<double, 6, 6> const & sqrt_info) {
    return new ceres::AutoDiffCostFunction<AutoDiffBetweenFactor, 6, 4, 3, 4, 3>(
        new AutoDiffBetweenFactor{a_R_b, a_t_b, sqrt_info});
  }

 private:
  Eigen::Quaterniond a_R_b_;
  Eigen::Vector3d a_t_b_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

Eigen::Matrix<double, 6, 1> ComputeResiduals(
    Eigen::Quaterniond const & qa, Eigen::Vector3d const & ta, Eigen::Quaterniond const & qb,
    Eigen::Vector3d const & tb, Eigen::Quaterniond const & qz, Eigen::Vector3d const & tz,
    Eigen::Matrix<double, 6, 6> const & sqrt_info) {
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = Sophus::SO3d{qb.inverse() * qa * qz}.log();
  error.tail<3>() = qb.inverse() * (qa * tz + ta - tb);

  return sqrt_info * error;
}

}  // namespace

TEST(tmp, tmp) {
  Eigen::Quaterniond const qa{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d const ta{Eigen::Vector3d::Random()};
  Eigen::Quaterniond const qb{Eigen::Quaterniond::UnitRandom()};
  Eigen::Vector3d const tb{Eigen::Vector3d::Random()};
  std::array<double const *, 4> const parameters{qa.coeffs().data(), ta.data(), qb.coeffs().data(),
                                                 tb.data()};

  Eigen::Matrix<double, 6, 1> residuals;

  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dwa_analytic;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_analytic;
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dwb_analytic;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_analytic;
  std::array<double *, 4> jacobians{dr_dwa_analytic.data(), dr_dta_analytic.data(),
                                    dr_dwb_analytic.data(), dr_dtb_analytic.data()};

  Eigen::Quaterniond const qz{qa.inverse() * qb};
  Eigen::Vector3d const tz{qa.inverse() * (tb - ta)};
  Eigen::Matrix<double, 6, 6> sqrt_info{Eigen::Matrix<double, 6, 6>::Identity()};
  BetweenFactor const factor{qz, tz, sqrt_info};

  factor.Evaluate(parameters.data(), residuals.data(), jacobians.data());
  std::cout << dr_dwa_analytic << std::endl;
  std::cout << std::endl;
  // std::cout << dr_dta_analytic << std::endl;
  // std::cout << std::endl;
  // std::cout << dr_dwb_analytic << std::endl;
  // std::cout << std::endl;
  // std::cout << dr_dtb_analytic << std::endl;
  // std::cout << std::endl;

  Eigen::Vector3d const dw{Eigen::Vector3d::Random().normalized() * 1e-9};
  Eigen::Quaterniond const dqx{
      Sophus::SO3d::exp(Eigen::Vector3d::UnitX() * dw.x()).unit_quaternion()};
  Eigen::Quaterniond const dqy{
      Sophus::SO3d::exp(Eigen::Vector3d::UnitY() * dw.y()).unit_quaternion()};
  Eigen::Quaterniond const dqz{
      Sophus::SO3d::exp(Eigen::Vector3d::UnitZ() * dw.z()).unit_quaternion()};
  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dwa_numeric;
  dr_dwa_numeric.setZero();
  dr_dwa_numeric.col(0) =
      (ComputeResiduals(qa * dqx, ta, qb, tb, qz, tz, sqrt_info) - residuals) / dw.x();
  dr_dwa_numeric.col(1) =
      (ComputeResiduals(qa * dqy, ta, qb, tb, qz, tz, sqrt_info) - residuals) / dw.y();
  dr_dwa_numeric.col(2) =
      (ComputeResiduals(qa * dqz, ta, qb, tb, qz, tz, sqrt_info) - residuals) / dw.z();
  // std::cout << dr_dwa_numeric << std::endl;
  EXPECT_TRUE(dr_dwa_analytic.isApprox(dr_dwa_numeric, 1e-5));

  Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dwb_numeric;
  dr_dwb_numeric.setZero();
  dr_dwb_numeric.col(0) =
      (ComputeResiduals(qa, ta, qb * dqx, tb, qz, tz, sqrt_info) - residuals) / dw.x();
  dr_dwb_numeric.col(1) =
      (ComputeResiduals(qa, ta, qb * dqy, tb, qz, tz, sqrt_info) - residuals) / dw.y();
  dr_dwb_numeric.col(2) =
      (ComputeResiduals(qa, ta, qb * dqz, tb, qz, tz, sqrt_info) - residuals) / dw.z();
  // std::cout << dr_dwb_numeric << std::endl;
  EXPECT_TRUE(dr_dwb_analytic.isApprox(dr_dwb_numeric, 1e-5));

  Eigen::Vector3d const dt{Eigen::Vector3d::Random().normalized() * 1e-9};
  Eigen::Vector3d const dtx{Eigen::Vector3d::UnitX() * dt.x()};
  Eigen::Vector3d const dty{Eigen::Vector3d::UnitY() * dt.y()};
  Eigen::Vector3d const dtz{Eigen::Vector3d::UnitZ() * dt.z()};
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_numeric;
  dr_dta_numeric.col(0) =
      (ComputeResiduals(qa, ta + dtx, qb, tb, qz, tz, sqrt_info) - residuals) / dt.x();
  dr_dta_numeric.col(1) =
      (ComputeResiduals(qa, ta + dty, qb, tb, qz, tz, sqrt_info) - residuals) / dt.y();
  dr_dta_numeric.col(2) =
      (ComputeResiduals(qa, ta + dtz, qb, tb, qz, tz, sqrt_info) - residuals) / dt.z();
  EXPECT_TRUE(dr_dta_analytic.isApprox(dr_dta_numeric, 1e-5));
  // std::cout << dr_dta_numeric << std::endl;

  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_numeric;
  dr_dtb_numeric.col(0) =
      (ComputeResiduals(qa, ta, qb, tb + dtx, qz, tz, sqrt_info) - residuals) / dt.x();
  dr_dtb_numeric.col(1) =
      (ComputeResiduals(qa, ta, qb, tb + dty, qz, tz, sqrt_info) - residuals) / dt.y();
  dr_dtb_numeric.col(2) =
      (ComputeResiduals(qa, ta, qb, tb + dtz, qz, tz, sqrt_info) - residuals) / dt.z();
  EXPECT_TRUE(dr_dtb_analytic.isApprox(dr_dtb_numeric, 1e-5));
  // std::cout << dr_dtb_numeric << std::endl;

  // Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqa_autodiff;
  // Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dta_autodiff;
  // Eigen::Matrix<double, 6, 4, Eigen::RowMajor> dr_dqb_autodiff;
  // Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dtb_autodiff;
  // Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dwa_autodiff;
  // Eigen::Matrix<double, 6, 3, Eigen::RowMajor> dr_dwb_autodiff;
  // jacobians = {dr_dqa_autodiff.data(), dr_dta_autodiff.data(), dr_dqb_autodiff.data(),
  //              dr_dtb_autodiff.data()};

  // auto const * manifold{new ceres::EigenQuaternionManifold};
  // auto const * autodiff_factor = AutoDiffBetweenFactor::Create(qz, tz, sqrt_info);
  // autodiff_factor->Evaluate(parameters.data(), residuals.data(), jacobians.data());

  // Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqa_dwa;
  // manifold->PlusJacobian(qa.coeffs().data(), dqa_dwa.data());
  // std::cout << dr_dqa_autodiff * dqa_dwa * .5 << std::endl;
  // std::cout << std::endl;

  // delete manifold;
  // delete autodiff_factor;
}
