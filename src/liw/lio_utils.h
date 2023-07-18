#ifndef LIO_UTILS_H_
#define LIO_UTILS_H_

#include "common/eigen_types.h"
#include "tools/imu.h"
#include "common/math_utils.h"
#include "tools/nav_state.h"
#include "tools/odom.h"

#include "common/cloudMap.hpp"

namespace zjloc
{
     enum IcpModel
     {
          POINT_TO_PLANE = 0,
          CT_POINT_TO_PLANE = 1
     };

     struct Neighborhood
     {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          Eigen::Vector3d center = Eigen::Vector3d::Zero();
          Eigen::Vector3d normal = Eigen::Vector3d::Zero();
          Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
          double a2D = 1.0; // Planarity coefficient
     };

     struct MeasureGroup
     {
          double lidar_begin_time_ = 0; // 雷达包的起始时间
          double lidar_end_time_ = 0;   // 雷达的终止时间
          std::vector<point3D> lidar_;  // 雷达点云
          std::deque<IMUPtr> imu_;      // 上一时时刻到现在的IMU读数
     };

     class state
     {
     public:
          Eigen::Quaterniond rotation;
          Eigen::Vector3d translation;
          Eigen::Vector3d velocity;
          Eigen::Vector3d ba;
          Eigen::Vector3d bg;

          Eigen::Quaterniond rotation_begin;
          Eigen::Vector3d translation_begin;
          Eigen::Vector3d velocity_begin;
          Eigen::Vector3d ba_begin;
          Eigen::Vector3d bg_begin;

          state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_,
                const Eigen::Vector3d &velocity_, const Eigen::Vector3d &ba_,
                const Eigen::Vector3d &bg_);

          state(const state *state_temp, bool copy = false);

          void release();
     };

     class cloudFrame
     {
     public:
          double time_frame_begin; //  current frame front stamp
          double time_frame_end;   //    next frame front stamp

          int frame_id;

          state *p_state;

          std::vector<point3D> point_surf; //  global frame
          std::vector<point3D> const_surf; //  lidar frame

          std::vector<point3D> surf_keypoints;

          double dt_offset;

          bool success;

          cloudFrame(std::vector<point3D> &point_surf_, std::vector<point3D> &const_surf_,
                     state *p_state_);

          cloudFrame(cloudFrame *p_cloud_frame);

          void release();
     };
     class numType
     {
     public:
          template <typename Derived>
          static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
          {
               typedef typename Derived::Scalar Scalar_t;

               Eigen::Quaternion<Scalar_t> dq;
               Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
               half_theta /= static_cast<Scalar_t>(2.0);
               dq.w() = static_cast<Scalar_t>(1.0);
               dq.x() = half_theta.x();
               dq.y() = half_theta.y();
               dq.z() = half_theta.z();
               dq.normalize();
               return dq;
          }

          template <typename Derived>
          static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &mat)
          {
               Eigen::Matrix<typename Derived::Scalar, 3, 3> mat_skew;
               mat_skew << typename Derived::Scalar(0), -mat(2), mat(1),
                   mat(2), typename Derived::Scalar(0), -mat(0),
                   -mat(1), mat(0), typename Derived::Scalar(0);
               return mat_skew;
          }

          template <typename Derived>
          static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
          {
               return q;
          }

          template <typename Derived>
          static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
          {
               Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
               Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
               ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
               ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
               return ans;
          }

          template <typename Derived>
          static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
          {
               Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
               Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
               ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
               ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
               return ans;
          }
     };
}
#endif // LIO_UTILS_H_