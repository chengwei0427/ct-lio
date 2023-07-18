//
// Created by xiang on 2022/3/15.
//

#ifndef SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H
#define SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H

#include <pcl/filters/voxel_grid.h>
#include "cloudMap.hpp"
#include "tools/point_types.h"

namespace zjloc
{
    FullCloudPtr Convert3DtoCloud(const std::vector<point3D> &input)
    {
        FullCloudPtr cloud(new FullPointCloudType);
        for (auto pt : input)
        {
            FullPointType p;
            p.x = pt.raw_point[0];
            p.y = pt.raw_point[1];
            p.z = pt.raw_point[2];
            p.intensity = pt.intensity;
            cloud->points.template emplace_back(p);
        }
        cloud->width = input.size();
        return cloud;
    }
    /**
     * 其他类型点云转到PointType点云
     * 用的最多的是全量点云转到XYZI点云
     * @tparam PointT
     * @param input
     * @return
     */
    template <typename PointT = FullPointType>
    CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input)
    {
        CloudPtr cloud(new PointCloudType);
        for (auto &pt : input->points)
        {
            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            cloud->points.template emplace_back(p);
        }
        cloud->width = input->width;
        return cloud;
    }

    /// 对点云进行voxel filter,指定分辨率
    inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1)
    {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel.setInputCloud(cloud);

        CloudPtr output(new PointCloudType);
        voxel.filter(*output);
        return output;
    }

} // namespace zjloc

#endif // SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H
