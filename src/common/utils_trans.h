#ifndef UTILS_TRANS_H_
#define UTILS_TRANS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cloudMap.hpp"
pcl::PointXYZI point3DtoPCL(point3D p);
void point3DtoPCL(std::vector<point3D> v_point_temp, pcl::PointCloud<pcl::PointXYZI>::Ptr &p_cloud_temp, bool global = false);

#endif