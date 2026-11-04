#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// World/sim vector, double precision. Default-constructs to zero: the build
// defines EIGEN_INITIALIZE_MATRICES_BY_ZERO for every target (CMakeLists.txt),
// and components rely on it (`Vec3 pos;` is the origin).
using Vec3 = Eigen::Vector3d;
