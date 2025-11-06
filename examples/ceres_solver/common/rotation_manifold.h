#include <ceres/ceres.h>

#include "examples/ceres_solver/common/types.h"

template <RotationUpdateMode mode>
class RotationManifold : public ceres::Manifold {
 public:
  RotationManifold();
  ~RotationManifold() override;
  int AmbientSize() const override;
  int TangentSize() const override;
  bool Plus(double const * x, double const * delta, double * x_plus_delta) const override;
  bool PlusJacobian(double const * x, double * jacobian) const override;
  bool Minus(double const * y, double const * x, double * y_minus_x) const override;
  bool MinusJacobian(double const * x, double * jacobian) const override;
};
