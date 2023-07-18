#ifndef _UTILS_H_
#define _UTILS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include <opencv2/core/core.hpp>
// #include <nav_msgs/Odometry.h>
#include <chrono>

#define MY_IMUPREINTEGRATE_

const double DEG2RAD = M_PI / 180.0;
#define THETA_THRESHOLD 0.0001 // sin a = a

class Utils
{
     // global variables
public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     static Eigen::Vector3d gravity;
     // mathematics
public:
     static Eigen::Matrix3d skew(const Eigen::Vector3d &v);
     static Eigen::Vector3d RToso3(const Eigen::Matrix3d &R);
     static Eigen::Quaterniond RToq(const Eigen::Matrix3d &R);
     static Eigen::Matrix3d qToR(const Eigen::Quaterniond &q);
     static Eigen::Vector3d qToso3(const Eigen::Quaterniond &q);
     static Eigen::Quaterniond so3Toq(const Eigen::Vector3d &so3);
     static Eigen::Matrix3d so3ToR(const Eigen::Vector3d &so3);
     static Eigen::Matrix3d Jl_so3(const Eigen::Vector3d &so3);
     static Eigen::Matrix3d Jr_so3(const Eigen::Vector3d &so3);
     static Eigen::Matrix3d Jl_so3_inv(const Eigen::Vector3d &so3);
     static Eigen::Matrix3d Jr_so3_inv(const Eigen::Vector3d &so3);
     static Eigen::Matrix3d Jr_R_inv(const Eigen::Matrix3d &R);

     //[q]_L [p]_R = [p]_R [q]_L
     static Eigen::Matrix4d quatLeftMatrix(const Eigen::Quaterniond &q);
     static Eigen::Matrix4d quatRightMatrix(const Eigen::Quaterniond &q);
     static Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta);
     // static Eigen::Isometry3d odomToEigen(const nav_msgs::OdometryConstPtr &odom_msg);
     static Eigen::Isometry3d so3ToT(const Eigen::Vector3d &rotation, const Eigen::Vector3d &translation);
     static Eigen::Matrix3d normalizeR(const Eigen::Matrix3d &R_in);
}; // namespace utils

// class TicToc
// {
// public:
//      TicToc(std::string name_in);
//      void tic();
//      void toc(int freq);

// private:
//      std::chrono::time_point<std::chrono::system_clock> start, end;
//      unsigned long total_frame;
//      double total_time;
//      std::string name;
// };

#endif //_UTILS_H_