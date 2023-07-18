#include "common_utility.hpp"

void subSampleFrame0(std::vector<point3D> &frame, double size_voxel)
{
     std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
     for (int i = 0; i < (int)frame.size(); i++)
     {
          auto kx = static_cast<short>(frame[i].raw_point[0] / size_voxel);
          auto ky = static_cast<short>(frame[i].raw_point[1] / size_voxel);
          auto kz = static_cast<short>(frame[i].raw_point[2] / size_voxel);
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

void gridSampling0(const pcl::PointCloud<pcl::PointXYZI>::Ptr &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints, double size_voxel_subsampling)
{
     keypoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
     std::vector<point3D> frame_sub;
     frame_sub.resize(frame->size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          point3D pt;
          pt.raw_point = Eigen::Vector3d(frame->points[i].x, frame->points[i].y, frame->points[i].z);
          // pt.point = pt.raw_point;
          pt.intensity = frame->points[i].intensity;

          frame_sub[i] = pt;
     }
     // std::cout << "input :" << frame->size() << ", " << frame_sub.size() << ",vs:" << size_voxel_subsampling << std::endl;
     subSampleFrame0(frame_sub, size_voxel_subsampling);
     // keypoints.reserve(frame_sub.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          pcl::PointXYZI pt;
          pt.x = frame_sub[i].raw_point[0], pt.y = frame_sub[i].raw_point[1], pt.z = frame_sub[i].raw_point[2];
          pt.intensity = frame_sub[i].intensity;
          keypoints->push_back(pt);
     }
}