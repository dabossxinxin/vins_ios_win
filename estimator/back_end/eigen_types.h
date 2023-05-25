#pragma once

#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXXd;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatXXf;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecXd;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VecXf;

typedef Eigen::Matrix<double, 15, 15> Mat15x15d;
typedef Eigen::Matrix<double, 14, 14> Mat14x14d;
typedef Eigen::Matrix<double, 13, 13> Mat13x13d;
typedef Eigen::Matrix<double, 12, 12> Mat12x12d;
typedef Eigen::Matrix<double, 11, 11> Mat11x11d;
typedef Eigen::Matrix<double, 10, 10> Mat10x10d;
typedef Eigen::Matrix<double, 9, 9> Mat9x9d;
typedef Eigen::Matrix<double, 8, 8> Mat8x8d;
typedef Eigen::Matrix<double, 7, 7> Mat7x7d;
typedef Eigen::Matrix<double, 6, 6> Mat6x6d;
typedef Eigen::Matrix<double, 5, 5> Mat5x5d;
typedef Eigen::Matrix<double, 4, 4> Mat4x4d;
typedef Eigen::Matrix<double, 3, 3> Mat3x3d;
typedef Eigen::Matrix<double, 2, 2> Mat2x2d;
typedef Eigen::Matrix<double, 1, 1> Mat1x1d;

typedef Eigen::Matrix<float, 15, 15> Mat15x15f;
typedef Eigen::Matrix<float, 14, 14> Mat14x14f;
typedef Eigen::Matrix<float, 13, 13> Mat13x13f;
typedef Eigen::Matrix<float, 12, 12> Mat12x12f;
typedef Eigen::Matrix<float, 11, 11> Mat11x11f;
typedef Eigen::Matrix<float, 10, 10> Mat10x10f;
typedef Eigen::Matrix<float, 9, 9> Mat9x9f;
typedef Eigen::Matrix<float, 8, 8> Mat8x8f;
typedef Eigen::Matrix<float, 7, 7> Mat7x7f;
typedef Eigen::Matrix<float, 6, 6> Mat6x6f;
typedef Eigen::Matrix<float, 5, 5> Mat5x5f;
typedef Eigen::Matrix<float, 4, 4> Mat4x4f;
typedef Eigen::Matrix<float, 3, 3> Mat3x3f;
typedef Eigen::Matrix<float, 2, 2> Mat2x2f;
typedef Eigen::Matrix<float, 1, 1> Mat1x1f;

typedef Eigen::Matrix<double, 15, 1> Vec15d;
typedef Eigen::Matrix<double, 14, 1> Vec14d;
typedef Eigen::Matrix<double, 13, 1> Vec13d;
typedef Eigen::Matrix<double, 12, 1> Vec12d;
typedef Eigen::Matrix<double, 11, 1> Vec11d;
typedef Eigen::Matrix<double, 10, 1> Vec10d;
typedef Eigen::Matrix<double, 9, 1> Vec9d;
typedef Eigen::Matrix<double, 8, 1> Vec8d;
typedef Eigen::Matrix<double, 7, 1> Vec7d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 5, 1> Vec5d;
typedef Eigen::Matrix<double, 4, 1> Vec4d;
typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::Matrix<double, 2, 1> Vec2d;

typedef Eigen::Matrix<float, 15, 1> Vec15f;
typedef Eigen::Matrix<float, 14, 1> Vec14f;
typedef Eigen::Matrix<float, 13, 1> Vec13f;
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 11, 1> Vec11f;
typedef Eigen::Matrix<float, 10, 1> Vec10f;
typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<float, 8, 1> Vec8f;
typedef Eigen::Matrix<float, 7, 1> Vec7f;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 5, 1> Vec5f;
typedef Eigen::Matrix<float, 4, 1> Vec4f;
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;

typedef Eigen::Quaterniond Qd;
typedef Eigen::Quaternionf Qf;

typedef std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> VecVec2d;
typedef std::vector<Vec2f, Eigen::aligned_allocator<Vec2f>> VecVec2f;
typedef std::vector<Vec3d, Eigen::aligned_allocator<Vec3d>> VecVec3d;
typedef std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>> VecVec3f;
typedef std::vector<MatXXd, Eigen::aligned_allocator<MatXXd>> VecMatXXd;
typedef std::vector<MatXXf, Eigen::aligned_allocator<MatXXf>> VecMatXXf;

typedef std::map<unsigned long, MatXXd, std::less<unsigned long>, Eigen::aligned_allocator<MatXXd>> MapMatXXd;
typedef std::map<unsigned long, MatXXf, std::less<unsigned long>, Eigen::aligned_allocator<MatXXf>> MapMatXXf;

typedef Eigen::Matrix<double, 8, 10> Mat8x10d;
typedef Eigen::Matrix<double, 8, 3> Mat8x3d;
typedef Eigen::Matrix<double, 5, 3> Mat5x3d;
typedef Eigen::Matrix<double, 4, 3> Mat4x3d;
typedef Eigen::Matrix<double, 4, 2> Mat4x2d;
typedef Eigen::Matrix<double, 2, 3> Mat2x3d;
typedef Eigen::Matrix<double, 4, 9> Mat4x9d;
typedef Eigen::Matrix<double, 8, 9> Mat8x9d;
typedef Eigen::Matrix<double, 9, 4> Mat9x4d;
typedef Eigen::Matrix<double, 9, 8> Mat9x8d;
typedef Eigen::Matrix<double, 6, 6> Mat6x6d;
typedef Eigen::Matrix<double, 9, 6> Mat9x6d;
typedef Eigen::Matrix<double, 8, 1> Mat8x1d;
typedef Eigen::Matrix<double, 1, 8> Mat1x8d;
typedef Eigen::Matrix<double, 9, 1> Mat9x1d;
typedef Eigen::Matrix<double, 1, 9> Mat1x9d;
typedef Eigen::Matrix<double, 8, 4> Mat8x4d;
typedef Eigen::Matrix<double, 4, 8> Mat4x8d;