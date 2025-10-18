#include "examples/ceres_solver/common/pose3.hpp"

#include <gtest/gtest.h>
#include <iostream>
#include <span>


TEST(test, test) {
  std::array<int, 3> a{1,2,3};
  std::span<const int> b{a};
  for (int n : b) {
    std::cout << n << " ";
  }
  std::cout << std::endl;

  a[2] = 4;
  for (int n : b) {
    std::cout << n << " ";
  }
  std::cout << std::endl;

}
// TEST(Pose3Test, TestBasics) {
//   // uint64_t const id{0};
//   Eigen::Quaterniond const rotation{Eigen::Quaterniond::Identity()};
//   Eigen::Vector3d const translation{Eigen::Vector3d::Random()};

//   auto const pose{std::make_shared<Pose3d>()};
//   pose->SetRotation(rotation);
//   pose->SetTranslation(translation);

//   EXPECT_TRUE(pose->Rotation().isApprox(rotation));
//   EXPECT_TRUE(pose->Translation().isApprox(translation));
// }

// TEST(Pose3Test, TestMap) {
//   const std::array<double, 7> data{{0., 0., 0., 1., 1., 2., 3.}};
//   const auto* const pose{reinterpret_cast<const Pose3d*>(data.data())};
//   std::cout << pose->at(0) << std::endl;
//   std::cout << pose->at(1) << std::endl;
//   std::cout << pose->at(2) << std::endl;
//   std::cout << pose->at(3) << std::endl;
//   std::cout << pose->Rotation().coeffs().transpose() << std::endl;
//   // EXPECT_TRUE(pose->Rotation().isApprox(Eigen::Quaterniond::Identity()));
//   // std::cout << pose->Translation().transpose() << std::endl;
//   SUCCEED();
// }
