#ifndef COMMON_UTILITY_HPP_
#define COMMON_UTILITY_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tr1/unordered_map>
#include "common/cloudMap.hpp"

struct VelodynePointXYZIRT
{
     PCL_ADD_POINT4D
     PCL_ADD_INTENSITY;
     uint16_t ring;
     float time;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct OusterPointXYZIRT
{
     PCL_ADD_POINT4D;
     float intensity;
     uint32_t t;
     uint16_t reflectivity;
     uint8_t ring;
     // uint16_t noise;
     uint16_t ambient;
     uint32_t range;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint16_t, ambient, ambient /*noise, noise*/)(uint32_t, range, range))

struct rsPointXYZIRT
{
     PCL_ADD_POINT4D;
     uint8_t intensity;
     uint16_t ring = 0;
     double timestamp = 0;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(rsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

typedef pcl::PointXYZI PointType;

void subSampleFrame0(std::vector<point3D> &frame, double size_voxel);

void gridSampling0(const pcl::PointCloud<pcl::PointXYZI>::Ptr &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints, double size_voxel_subsampling);

#endif