#pragma once

#include <ceres/ceres.h>

struct Pose3Manifold : public ceres::Manifold {
  static constexpr std::size_t kAmbientRotationIndex{0};
  static constexpr std::size_t kAmbientRotationSize{4};
  static constexpr std::size_t kTangentRotationIndex{0};
  static constexpr std::size_t kTangentRotationSize{3};

  static constexpr std::size_t kAmbientTranslationIndex{kAmbientRotationIndex +
                                                        kAmbientRotationSize};
  static constexpr std::size_t kAmbientTranslationSize{3};
  static constexpr std::size_t kTangentTranslationIndex{kTangentRotationIndex +
                                                        kTangentRotationSize};
  static constexpr std::size_t kTangentTranslationSize{3};

  static constexpr std::size_t kAmbientSize{kAmbientRotationSize + kAmbientTranslationSize};
  static constexpr std::size_t kTangentSize{kTangentRotationSize + kTangentTranslationSize};
};

class Pose3AnalyticDiffManifold : public Pose3Manifold {
 public:
  Pose3AnalyticDiffManifold();
  ~Pose3AnalyticDiffManifold();
  int AmbientSize() const override;
  int TangentSize() const override;
  bool Plus(double const * x, double const * delta, double * x_plus_delta) const override;
  bool PlusJacobian(double const * x, double * jacobian) const override;
  bool RightMultiplyByPlusJacobian(double const * x, int const num_rows,
                                   double const * ambient_matrix,
                                   double * tangent_matrix) const override;
  bool Minus(double const * y, double const * x, double * y_minus_x) const override;
  bool MinusJacobian(double const * x, double * jacobian) const override;
};
