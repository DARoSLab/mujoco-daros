/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include "cTypes.h"
#include <eigen3/Eigen/Dense>

// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 1x1 Vector
template <typename T>
using Vec1 = typename Eigen::Matrix<T, 1, 1>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 5x1 Vector
template <typename T>
using Vec5 = typename Eigen::Matrix<T, 5, 1>;

// 6x1 Vector
template <typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

// 7x1 Vector
template <typename T>
using Vec7 = typename Eigen::Matrix<T, 7, 1>;

// 8x1 Vector
template <typename T>
using Vec8 = typename Eigen::Matrix<T, 8, 1>;

// 8x1 Vector
template <typename T>
using Vec9 = typename Eigen::Matrix<T, 9, 1>;

// 10x1 Vector
template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 12x1 Vector
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 13x1 Vector
template <typename T>
using Vec13 = Eigen::Matrix<T, 13, 1>;


// 18x1 Vector
template <typename T>
using Vec16 = Eigen::Matrix<T, 16, 1>;

// 18x1 Vector
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

// 25x1 Vector
template <typename T>
using Vec25 = Eigen::Matrix<T, 25, 1>;

// 28x1 vector
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 100x1 vector
template <typename T>
using Vec100 = Eigen::Matrix<T, 100, 1>;

// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 2x2 Matrix
template <typename T>
using Mat2 = typename Eigen::Matrix<T, 2, 2>;

// 5x5 Matrix
template <typename T>
using Mat5 = typename Eigen::Matrix<T, 5, 5>;

// 4x1 Vector (w, x, y, z)
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template <typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 3x12 Matrix
template <typename T>
using Mat312 = typename Eigen::Matrix<T, 3, 12>;

// 12x12 Matrix
template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 13x13 Matrix
template <typename T>
using Mat13 = typename Eigen::Matrix<T, 13, 13>;

// 13x12 Matrix
template <typename T>
using Mat1312 = typename Eigen::Matrix<T, 13, 12>;

// 18x18 Matrix
template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 Matrix
template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 Matrix
template <typename T>
using Mat32 = Eigen::Matrix<T, 3, 2>;

// 3x4 Matrix
template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

// 4x2 Matrix
template <typename T>
using Mat42 = Eigen::Matrix<T, 4, 2>;

// 4x3 Matrix
template <typename T>
using Mat43 = Eigen::Matrix<T, 4, 3>;

// 2x3 Matrix
template <typename T>
using Mat23 = Eigen::Matrix<T, 2, 3>;

// 4x4 Matrix
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 10x1 Vector
template <typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with spatial vector columns
template <typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

// Dynamically sized vector of integers
using DiVec = typename Eigen::Matrix<int, Eigen::Dynamic, 1>;

// std::vector (a list) of Eigen things
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;


struct IMU_Data {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  Quat<float> quat;
  // todo is there status for the vectornav?
};

struct CheaterState {
  Quat<double> orientation;
  Vec3<double> position;
  Vec3<double> omegaBody;
  Vec3<double> vBody;
  Vec3<double> acceleration;
};

#endif  // PROJECT_CPPTYPES_H
