#include "Eigen/Core"
#include "Eigen/Geometry"

class Constants {
 public:
  Constants() = delete;
  ~Constants() = delete;

  static constexpr std::size_t kPoseErrorSize{6};
};

template <typename T>
bool EvaluateRightOplus(
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
    Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb, Eigen::Quaternion<T> const & a_qm_b,
    Eigen::Matrix<T, 3, 1> const & a_tm_ab,
    Eigen::Matrix<T, Constants::kPoseErrorSize, Constants::kPoseErrorSize> sqrt_info,
    Eigen::Map<Eigen::Matrix<T, Constants::kPoseErrorSize, 1>> & whitened_error, T ** jacobians);

template <typename T>
static bool EvaluateLeftOplus(Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_a,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_ra,
                              Eigen::Map<Eigen::Quaternion<T> const> const & r_qe_b,
                              Eigen::Map<Eigen::Matrix<T, 3, 1> const> const & r_te_rb,
                              Eigen::Quaternion<T> const & a_qm_b,
                              Eigen::Matrix<T, 3, 1> const & a_tm_ab,
                              Eigen::Matrix<T, Constants::kPoseErrorSize, Constants::kPoseErrorSize> sqrt_info,
                              Eigen::Map<Eigen::Matrix<T, Constants::kPoseErrorSize, 1>> & whitened_error,
                              T ** jacobians);
