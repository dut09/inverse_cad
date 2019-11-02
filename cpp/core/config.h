#ifndef CORE_CONFIG_H
#define CORE_CONFIG_H

// Common headers.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <sys/time.h>
// Eigen.
#include "Eigen/Dense"
using real = double;

#define SAFETY_CHECK    1

using Vector2i = Eigen::Matrix<int, 2, 1>;
using Vector3i = Eigen::Matrix<int, 3, 1>;
using Vector4i = Eigen::Matrix<int, 4, 1>;
using Vector2r = Eigen::Matrix<real, 2, 1>;
using Vector3r = Eigen::Matrix<real, 3, 1>;
using Vector4r = Eigen::Matrix<real, 4, 1>;
using VectorXr = Eigen::Matrix<real, -1, 1>;

#endif
