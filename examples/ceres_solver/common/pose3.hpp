#pragma once

#include "examples/ceres_solver/common/pose3.h"

template <typename T>
Pose3<T>::~Pose3() = default;

template <typename T>
Pose3<T>::Pose3(uint64_t id)
    : SizedParameterBlock<T, 7>(id),
      rotation_{this->template ParameterPtr(kRotationIndex)},
      translation_{this->template ParameterPtr(kTranslationIndex)} {}

template <typename T>
void Pose3<T>::SetRotation(Eigen::Quaternion<T> const & rotation) {
  rotation_ = rotation;
}

template <typename T>
void Pose3<T>::SetTranslation(Eigen::Matrix<T, 3, 1> const & translation) {
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
