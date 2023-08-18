#ifndef LIDAR_ODOM_H__
#define LIDAR_ODOM_H__
#include "common/timer/timer.h"
#include "liw/lidarFactor.h"
#include "liw/poseParameterization.h"

#include "algo/eskf.hpp"
#include "algo/static_imu_init.h"
#include "liw/lio_utils.h"

#include <condition_variable>

#include "tools/tool_color_printf.hpp"
#include "common/timer/timer.h"
#include "common/utility.h"
#include "tools/point_types.h"

#include <sys/times.h>
#include <sys/vtimes.h>

namespace zjloc
{

     struct liwOptions
     {
          double surf_res;
          bool log_print;
          int max_num_iteration;
          //   ct_icp
          IcpModel icpmodel;
          double size_voxel_map;
          double min_distance_points;
          int max_num_points_in_voxel;
          double max_distance;
          double weight_alpha;
          double weight_neighborhood;
          double max_dist_to_plane_icp;
          int init_num_frames;
          int voxel_neighborhood;
          int max_number_neighbors;
          int threshold_voxel_occupancy;
          bool estimate_normal_from_neighborhood;
          int min_number_neighbors;
          double power_planarity;
          int num_closest_neighbors;

          double sampling_rate;

          double ratio_of_nonground;
          int max_num_residuals;
          int min_num_residuals;

          MotionCompensation motion_compensation;
          bool point_to_plane_with_distortion = false;
          double beta_location_consistency;    // Constraints on location
          double beta_constant_velocity;       // Constraint on velocity
          double beta_small_velocity;          // Constraint on the relative motion
          double beta_orientation_consistency; // Constraint on the orientation consistency

          double thres_orientation_norm;
          double thres_translation_norm;
     };

     class lidarodom
     {
     public:
          lidarodom(/* args */);
          ~lidarodom();

          bool init(const std::string &config_yaml);

          void pushData(std::vector<point3D>, std::pair<double, double> data);
          void pushData(IMUPtr imu);

          void run();

          int getIndex() { return index_frame; }

          void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun) { pub_cloud_to_ros = fun; }
          void setFunc(std::function<bool(std::string &topic_name, SE3 &pose, double time)> &fun) { pub_pose_to_ros = fun; }
          void setFunc(std::function<bool(std::string &topic_name, double time1, double time2)> &fun) { pub_data_to_ros = fun; }

     private:
          void loadOptions();
          /// 使用IMU初始化
          void TryInitIMU();

          /// 利用IMU预测状态信息
          /// 这段时间的预测数据会放入imu_states_里
          void Predict();

          /// 对measures_中的点云去畸变
          void Undistort(std::vector<point3D> &points);

          std::vector<MeasureGroup> getMeasureMents();

          /// 处理同步之后的IMU和雷达数据
          void ProcessMeasurements(MeasureGroup &meas);

          void stateInitialization();

          cloudFrame *buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                 double timestamp_begin, double timestamp_end);

          void poseEstimation(cloudFrame *p_frame);

          void optimize(cloudFrame *p_frame);

          void lasermap_fov_segment();

          void map_incremental(cloudFrame *p_frame, int min_num_points = 0);

          void addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                             const double &intensity, double voxel_size,
                             int max_num_points_in_voxel, double min_distance_points,
                             int min_num_points, cloudFrame *p_frame);

          void addSurfCostFactor(std::vector<ceres::CostFunction *> &surf, std::vector<Eigen::Vector3d> &normals,
                                 std::vector<point3D> &keypoints, const cloudFrame *p_frame);

          void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points,
                             const Eigen::Vector3d &point, const double &intensity,
                             cloudFrame *p_frame);

          double checkLocalizability(std::vector<Eigen::Vector3d> planeNormals);

          // search neighbors
          Neighborhood computeNeighborhoodDistribution(
              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                          int nb_voxels_visited, double size_voxel_map, int max_num_neighbors,
                          int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);

          inline Sophus::SO3d r2SO3(const Eigen::Vector3d r)
          {
               return Sophus::SO3d::exp(r);
          }

     private:
          /// @brief 数据
          std::string config_yaml_;
          StaticIMUInit imu_init_;    // IMU静止初始化
          SE3 TIL_;                   //   lidar 转换到 imu
          MeasureGroup measures_;     // sync IMU and lidar scan
          bool imu_need_init_ = true; // 是否需要估计IMU初始零偏
          int index_frame = 1;
          liwOptions options_;

          voxelHashMap voxel_map;
          Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); //   lidar 转换到 imu坐标系下
          Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();     //   need init
          double laser_point_cov = 0.001;

          double PR_begin[6] = {0, 0, 0, 0, 0, 0}; //  p         r
          double PR_end[6] = {0, 0, 0, 0, 0, 0};   //  p         r

          /// @brief 滤波器
          ESKFD eskf_;
          std::vector<NavStated> imu_states_; // ESKF预测期间的状态
          IMUPtr last_imu_ = nullptr;

          double time_curr;
          double delay_time_;
          Vec3d g_{0, 0, -9.8};

          /// @brief 数据管理及同步
          std::deque<std::vector<point3D>> lidar_buffer_;
          std::deque<IMUPtr> imu_buffer_;    // imu数据缓冲
          double last_timestamp_imu_ = -1.0; // 最近imu时间
          double last_timestamp_lidar_ = 0;  // 最近lidar时间
          std::deque<std::pair<double, double>> time_buffer_;

          /// @brief mutex
          std::mutex mtx_buf;
          std::mutex mtx_state;
          std::condition_variable cond;

          state *current_state;
          std::vector<cloudFrame *> all_cloud_frame; //  cache all frame
          std::vector<state *> all_state_frame;      //   多保留一份state，这样可以不用去缓存all_cloud_frame

          std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> pub_cloud_to_ros;
          std::function<bool(std::string &topic_name, SE3 &pose, double time)> pub_pose_to_ros;
          std::function<bool(std::string &topic_name, double time1, double time2)> pub_data_to_ros;
          pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;
     };
}

#endif