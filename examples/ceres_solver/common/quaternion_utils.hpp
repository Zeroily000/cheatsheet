#pragma once;

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Matrix4d quaternionLeftMultiplicationMatrix(Eigen::Quaterniond const & quaternion);

Eigen::Matrix4d quaternionRightMultiplicationMatrix(Eigen::Quaterniond const & quaternion);
