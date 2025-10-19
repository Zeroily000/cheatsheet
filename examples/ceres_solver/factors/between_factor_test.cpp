#include "examples/ceres_solver/factors/between_factor.hpp"

#include <gtest/gtest.h>
#include <iostream>

TEST(tmp, tmp) {
  Eigen::Quaterniond qa{Eigen::Quaterniond::UnitRandom()};
  std::cout << qa.coeffs().transpose() << std::endl;
  EXPECT_EQ(qa.norm(), 1.);
}
