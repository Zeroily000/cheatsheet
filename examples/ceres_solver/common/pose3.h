#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "examples/ceres_solver/common/pose3_manifolds.hpp"
#include "examples/ceres_solver/common/sized_parameter_block.hpp"

template <typename T>
class Pose3 : public SizedParameterBlock<T, Pose3Manifold::kAmbientSize> {
// class Pose3 : public std::array<T, Pose3Manifold::kAmbientSize> {
 public:
  // Pose3(uint64_t id);
  Pose3();
  ~Pose3();

  Pose3(Pose3 const &) = delete;
  Pose3(Pose3 &&) noexcept = delete;
  Pose3 & operator=(Pose3 const &) = delete;
  Pose3 & operator=(Pose3 &&) noexcept = delete;

  void SetRotation(Eigen::Quaternion<T> const & rotation);

  void SetTranslation(Eigen::Matrix<T, 3, 1> const & translation);

  Eigen::Map<Eigen::Quaternion<T>> const & Rotation() const;

  Eigen::Map<Eigen::Matrix<T, 3, 1>> const & Translation() const;

 private:
  Eigen::Map<Eigen::Quaternion<T>> rotation_;
  Eigen::Map<Eigen::Matrix<T, 3, 1>> translation_;
};
