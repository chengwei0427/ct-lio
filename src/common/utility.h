#ifndef UTILITY_H_
#define UTILITY_H_
// c++
#include <iostream>
#include <string>
#include <tr1/unordered_map>

#include "cloudMap.hpp"

#include "sophus/so3.hpp"

enum MotionCompensation
{
     NONE = 0,
     CONSTANT_VELOCITY = 1,
     ITERATIVE = 2,
     CONTINUOUS = 3
};

double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb);

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b);

void subSampleFrame(std::vector<point3D> &frame, double size_voxel);

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

void distortFrame(std::vector<point3D> &points, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end,
                  Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

void transformPoint(MotionCompensation compensation, point3D &point_temp, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end,
                    Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

namespace std
{
     template <typename T, typename... Args>
     std::unique_ptr<T> make_unique(Args &&...args)
     {
          return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
     }
}
#endif