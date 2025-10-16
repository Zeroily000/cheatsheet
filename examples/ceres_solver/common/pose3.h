#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "examples/ceres_solver/common/sized_parameter_block.hpp"

template <typename T>
class Pose3 : public SizedParameterBlock<T, 7> {
 public:
  ~Pose3() override;

  static std::shared_ptr<Pose3> Create(uint64_t id, Eigen::Quaternion<T> const & rotation,
                                       Eigen::Matrix<T, 3, 1> const & translation);

  Eigen::Map<Eigen::Quaternion<T>> const & Rotation() const;

  Eigen::Map<Eigen::Matrix<T, 3, 1>> const & Translation() const;

 private:
  static constexpr std::size_t kRotationIndex = 0;
  static constexpr std::size_t kRotationSize = 4;
  static constexpr std::size_t kTranslationIndex = kRotationIndex + kRotationSize;
  static constexpr std::size_t kTranslationSize = 3;
  // static_assert(kRotationSize + kTranslationSize == kSize);

  Pose3(uint64_t id, Eigen::Quaternion<T> const & rotation,
        Eigen::Matrix<T, 3, 1> const & translation);

  Eigen::Map<Eigen::Quaternion<T>> rotation_;
  Eigen::Map<Eigen::Matrix<T, 3, 1>> translation_;
};
