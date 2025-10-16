#include "examples/ceres_solver/common/pose3_manifolds.hpp"

Pose3AnalyticDiffManifold::Pose3AnalyticDiffManifold() = default;

Pose3AnalyticDiffManifold::~Pose3AnalyticDiffManifold() = default;

int Pose3AnalyticDiffManifold::AmbientSize() const { return kAmbientSize; }

int Pose3AnalyticDiffManifold::TangentSize() const { return kTangentSize; }

bool Pose3AnalyticDiffManifold::Plus(double const * x, double const * delta,
                                     double * x_plus_delta) const {
  return false;
}

bool Pose3AnalyticDiffManifold::PlusJacobian(double const * x, double * jacobian) const {
  return false;
}

bool Pose3AnalyticDiffManifold::RightMultiplyByPlusJacobian(double const * x, int const num_rows,
                                                            double const * ambient_matrix,
                                                            double * tangent_matrix) const {
  return false;
}

bool Pose3AnalyticDiffManifold::Minus(double const * y, double const * x,
                                      double * y_minus_x) const {
  return false;
}

bool Pose3AnalyticDiffManifold::MinusJacobian(double const * x, double * jacobian) const {
  return false;
}
