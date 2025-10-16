#include "examples/ceres_solver/common/pose3.hpp"

#include <gtest/gtest.h>

TEST(Pose3Test, TestBasics) {
  const uint64_t id{0};
  const Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  const Eigen::Vector3d translation{Eigen::Vector3d::Random()};

  const std::shared_ptr<Pose3d> pose{Pose3d::Create(id, rotation, translation)};

  EXPECT_TRUE(pose->Rotation().isApprox(rotation));
  EXPECT_TRUE(pose->Translation().isApprox(translation));
}
