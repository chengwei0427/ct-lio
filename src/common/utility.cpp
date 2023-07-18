#include "utility.h"

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb)
{
     double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
     norm = std::acos(norm) * 180 / M_PI;
     return norm;
}

double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb)
{
     Eigen::Quaterniond q_a = Eigen::Quaterniond(Sophus::SO3d::exp(qa).matrix());
     Eigen::Quaterniond q_b = Eigen::Quaterniond(Sophus::SO3d::exp(qb).matrix());
     q_a.normalize(), q_b.normalize();

     Eigen::Matrix3d rota = q_a.toRotationMatrix();
     Eigen::Matrix3d rotb = q_b.toRotationMatrix();

     double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
     norm = std::acos(norm) * 180 / M_PI;
     return norm;
}

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b)
{
     Eigen::Matrix3d rota = q_a.toRotationMatrix();
     Eigen::Matrix3d rotb = q_b.toRotationMatrix();

     double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
     norm = std::acos(norm) * 180 / M_PI;
     return norm;
}

void subSampleFrame(std::vector<point3D> &frame, double size_voxel)
{
     std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
     for (int i = 0; i < (int)frame.size(); i++)
     {
          auto kx = static_cast<short>(frame[i].point[0] / size_voxel);
          auto ky = static_cast<short>(frame[i].point[1] / size_voxel);
          auto kz = static_cast<short>(frame[i].point[2] / size_voxel);
          grid[voxel(kx, ky, kz)].push_back(frame[i]);
     }
     frame.resize(0);
     int step = 0;
     for (const auto &n : grid)
     {
          if (n.second.size() > 0)
          {
               frame.push_back(n.second[0]);
               step++;
          }
     }
}

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)
{
     keypoints.resize(0);
     std::vector<point3D> frame_sub;
     frame_sub.resize(frame.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          frame_sub[i] = frame[i];
     }
     subSampleFrame(frame_sub, size_voxel_subsampling);
     keypoints.reserve(frame_sub.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          keypoints.push_back(frame_sub[i]);
     }
}

void distortFrame(std::vector<point3D> &points, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end,
                  Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
     Eigen::Quaterniond q_end_inv = q_end.inverse();         // Rotation of the inverse pose
     Eigen::Vector3d t_end_inv = -1.0 * (q_end_inv * t_end); // Translation of the inverse pose
     for (auto &point_temp : points)
     {
          double alpha_time = point_temp.alpha_time;
          Eigen::Quaterniond q_alpha = q_begin.slerp(alpha_time, q_end);
          q_alpha.normalize();
          Eigen::Vector3d t_alpha = (1.0 - alpha_time) * t_begin + alpha_time * t_end;

          point_temp.raw_point = R_imu_lidar.transpose() *
                                     (q_end_inv * (q_alpha * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t_alpha) + t_end_inv) -
                                 R_imu_lidar.transpose() * t_imu_lidar;
     }
}

void transformPoint(MotionCompensation compensation, point3D &point_temp, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end,
                    Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
     Eigen::Vector3d t;
     Eigen::Matrix3d R;
     double alpha_time = point_temp.alpha_time;
     switch (compensation)
     {
     case MotionCompensation::NONE:
     case MotionCompensation::CONSTANT_VELOCITY:
          R = q_end.toRotationMatrix();
          t = t_end;
          break;
     case MotionCompensation::CONTINUOUS:
     case MotionCompensation::ITERATIVE:
          R = q_begin.slerp(alpha_time, q_end).normalized().toRotationMatrix();
          t = (1.0 - alpha_time) * t_begin + alpha_time * t_end;
          break;
     }
     point_temp.point = R * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t;
}