#pragma once
#include <Eigen/Dense>


constexpr int DOF = 6;  // [x, y, z, vx, vy, vz]
using StateVec = Eigen::Matrix<double, DOF, 1>;
using StateMat = Eigen::Matrix<double, DOF, DOF>;