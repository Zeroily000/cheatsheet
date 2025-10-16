#include "examples/ceres_solver/common/pose3.hpp"

#include <gtest/gtest.h>

TEST(Pose3Test, TestBasics) {
  uint64_t const id{0};
  Eigen::Quaterniond const rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d const translation{Eigen::Vector3d::Random()};

  auto const pose{std::make_shared<Pose3d>(id)};
  pose->SetRotation(rotation);
  pose->SetTranslation(translation);

  EXPECT_TRUE(pose->Rotation().isApprox(rotation));
  EXPECT_TRUE(pose->Translation().isApprox(translation));
}
