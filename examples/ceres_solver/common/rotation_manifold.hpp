#pragma once

#include <ceres/ceres.h>

class RotationManifold : public ceres::Manifold {
  public:
  static constexpr std::size_t kAmbientSize{4};
  static constexpr std::size_t kTangentSize{3};

  RotationManifold();
  ~RotationManifold();
  int AmbientSize() const override;
  int TangentSize() const override;
  bool Plus(double const * x, double const * delta, double * x_plus_delta) const override;
  bool PlusJacobian(double const * x, double * jacobian) const override;
  bool Minus(double const * y, double const * x, double * y_minus_x) const override;
  bool MinusJacobian(double const * x, double * jacobian) const override;
};
