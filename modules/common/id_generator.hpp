#pragma once

#include "modules/common/id_generator.h"

namespace common {

template <typename T, std::enable_if_t<std::is_integral_v<T>, std::nullptr_t> Dummy>
IdGenerator<T, Dummy>::IdGenerator(id_t id) : id_(id) {}

template <typename T, std::enable_if_t<std::is_integral_v<T>, std::nullptr_t> Dummy>
IdGenerator<T, Dummy>::id_t IdGenerator<T, Dummy>::operator()() {
  return id_++;
}

}  // namespace common
