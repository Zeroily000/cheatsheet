#pragma once

#include "examples/ceres_solver/common/pose3.h"

template <typename T>
Pose3<T>::~Pose3() = default;

template <typename T>
std::shared_ptr<Pose3<T>> Pose3<T>::Create(uint64_t id, Eigen::Quaternion<T> const & rotation,
                                           Eigen::Matrix<T, 3, 1> const & translation) {
  return std::shared_ptr<Pose3<T>>{new Pose3<T>{id, rotation, translation}};
}

template <typename T>
Pose3<T>::Pose3(uint64_t id, Eigen::Quaternion<T> const & rotation,
                Eigen::Matrix<T, 3, 1> const & translation)
    : SizedParameterBlock<T, 7>(id),
      rotation_{this->template ParameterPtr(kRotationIndex)},
      translation_{this->template ParameterPtr(kTranslationIndex)} {
  rotation_ = rotation;
  translation_ = translation;
}

template <typename T>
Eigen::Map<Eigen::Quaternion<T>> const & Pose3<T>::Rotation() const {
  return rotation_;
}

template <typename T>
Eigen::Map<Eigen::Matrix<T, 3, 1>> const & Pose3<T>::Translation() const {
  return translation_;
}

using Pose3d = Pose3<double>;
