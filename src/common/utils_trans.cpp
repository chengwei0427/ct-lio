#include "utils_trans.h"

pcl::PointXYZI point3DtoPCL(point3D p)
{
     pcl::PointXYZI pt;
     pt.x = p.point[0];
     pt.y = p.point[1];
     pt.z = p.point[2];
     pt.intensity = p.intensity;
     return pt;
}

void point3DtoPCL(std::vector<point3D> v_point_temp, pcl::PointCloud<pcl::PointXYZI>::Ptr &p_cloud_temp, bool global)
{
     for (int i = 0; i < v_point_temp.size(); i++)
     {
          pcl::PointXYZI cloud_temp;
          if (global)
          {
               cloud_temp.x = v_point_temp[i].point.x();
               cloud_temp.y = v_point_temp[i].point.y();
               cloud_temp.z = v_point_temp[i].point.z();
          }
          else
          {
               cloud_temp.x = v_point_temp[i].raw_point.x();
               cloud_temp.y = v_point_temp[i].raw_point.y();
               cloud_temp.z = v_point_temp[i].raw_point.z();
          }
          cloud_temp.intensity = v_point_temp[i].intensity;

          p_cloud_temp->points.push_back(cloud_temp);
     }
}