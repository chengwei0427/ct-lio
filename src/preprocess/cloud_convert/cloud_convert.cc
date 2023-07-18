#include "cloud_convert.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
// #include <execution>

namespace zjloc
{

    // void CloudConvert::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, FullCloudPtr &pcl_out) {
    //     AviaHandler(msg);
    //     *pcl_out = cloud_out_;
    // }

    void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                               std::vector<point3D> &pcl_out)
    {
        switch (lidar_type_)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }
        pcl_out = cloud_out_;
    }

    /*void CloudConvert::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        int plsize = msg->point_num;

        cloud_out_.reserve(plsize);
        cloud_full_.resize(plsize);

        std::vector<bool> is_valid_pt(plsize, false);
        std::vector<uint> index(plsize - 1);
        for (uint i = 0; i < plsize - 1; ++i)
        {
            index[i] = i + 1; // 从1开始
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i)
                      {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;
                cloud_full_[i].time = msg->points[i].offset_time / float(1000000);

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7)) {
                    is_valid_pt[i] = true;
                }
            }
        } });

        for (uint i = 1; i < plsize; i++)
        {
            if (is_valid_pt[i])
            {
                cloud_out_.points.push_back(cloud_full_[i]);
            }
        }
    }*/

    void CloudConvert::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        timespan_ = pl_orig.points.back().t / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
        //           << " , 100: " << pl_orig.points[100].t / tm_scale
        //           << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
            point_temp.intensity = pl_orig.points[i].intensity;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();
        //  FIXME:  时间戳大于0.1
        auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << timespan_ << std::endl;

        // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            // if (i % point_filter_num_ != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            // point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.timestamp = pl_orig.points[i].timestamp;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;
            if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
                std::cout << point_temp.alpha_time << ", this may error." << std::endl;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1e6; //   1e6 - nclt kaist or 1

        //  FIXME:  nclt 及kaist时间戳大于0.1
        auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
        while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().time / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 100: " << pl_orig.points[100].time / tm_scale << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        point_filter_num_ = yaml["preprocess"]["point_filter_num"].as<int>();

        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::VELO32;
            LOG(INFO) << "Using Velodyne 32 Lidar";
        }
        else if (lidar_type == 3)
        {
            lidar_type_ = LidarType::OUST64;
            LOG(INFO) << "Using OUST 64 Lidar";
        }
        else if (lidar_type == 4)
        {
            lidar_type_ = LidarType::ROBOSENSE16;
            LOG(INFO) << "Using Robosense 16 LIdar";
        }
        else
        {
            LOG(WARNING) << "unknown lidar_type";
        }
    }

} // namespace zjloc
