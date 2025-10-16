#pragma once

#include "examples/ceres_solver/common/sized_parameter_block.h"

#include <glog/logging.h>

template <typename T, std::size_t N>
SizedParameterBlock<T, N>::SizedParameterBlock(uint64_t id) : id_{id} {}

template <typename T, std::size_t N>
SizedParameterBlock<T, N>::~SizedParameterBlock() = default;

template <typename T, std::size_t N>
T * SizedParameterBlock<T, N>::ParameterPtr(std::size_t pos) {
  CHECK_LT(pos, parameters_.size());
  return parameters_.data() + pos;
}

template <typename T, std::size_t N>
T const * SizedParameterBlock<T, N>::ParameterPtr(std::size_t pos) const {
  return ParameterPtr(pos);
}
