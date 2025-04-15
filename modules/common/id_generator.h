#pragma once

#include <cstddef>
#include <type_traits>

namespace common {

template <typename T, std::enable_if_t<std::is_integral_v<T>, std::nullptr_t> = nullptr>
class IdGenerator {
  using id_t = std::remove_cv_t<T>;
 public:
  IdGenerator(id_t id);

  id_t operator()();

 private:
  id_t id_;
};

}  // namespace common
