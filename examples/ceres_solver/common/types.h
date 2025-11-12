#pragma once

enum class RotationUpdateMode {
  kRight,
  kLeft,
};

enum class JacobianComputationMethod {
  kAutomatic,
  kAnalytic,
};

template <typename T>
class Constants {
 public:
  static constexpr T kEpsilon{1e-9};

  Constants() = delete;
  ~Constants() = delete;
  Constants(Constants const &) = delete;
  Constants(Constants &&) noexcept = delete;
  Constants & operator=(Constants const &) = delete;
  Constants & operator=(Constants &&) noexcept = delete;
};
