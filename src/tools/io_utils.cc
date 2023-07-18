//
// Created by xiang on 2021/7/20.
//
#include "tools/io_utils.h"

#include <glog/logging.h>

namespace zjloc
{

    void TxtIO::Go()
    {
        if (!fin)
        {
            LOG(ERROR) << "未能找到文件";
            return;
        }

        while (!fin.eof())
        {
            std::string line;
            std::getline(fin, line);
            if (line.empty())
            {
                continue;
            }

            if (line[0] == '#')
            {
                // 以#开头的是注释
                continue;
            }

            // load data from line
            std::stringstream ss;
            ss << line;
            std::string data_type;
            ss >> data_type;

            if (data_type == "IMU" && imu_proc_)
            {
                double time, gx, gy, gz, ax, ay, az;
                ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
                // imu_proc_(IMU(time, Vec3d(gx, gy, gz) * math::kDEG2RAD, Vec3d(ax, ay, az)));
                imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
            }
            else if (data_type == "ODOM" && odom_proc_)
            {
                double time, wl, wr;
                ss >> time >> wl >> wr;
                odom_proc_(Odom(time, wl, wr));
            }
        }

        LOG(INFO) << "done.";
    }

    void RosbagIO::Go()
    {
        rosbag::Bag bag(bag_file_);
        LOG(INFO) << "running in " << bag_file_ << ", reg process func: " << process_func_.size();

        if (!bag.isOpen())
        {
            LOG(ERROR) << "cannot open " << bag_file_;
            return;
        }

        rosbag::View view(bag);

        for (const rosbag::MessageInstance &m : view)
        {
            auto iter = process_func_.find(m.getTopic());
            if (iter != process_func_.end())
            {
                iter->second(m);
            }

            if (global::FLAG_EXIT)
            {
                break;
            }
        }

        bag.close();

        LOG(INFO) << "bag " << bag_file_ << " finished.";
    }

    RosbagIO &RosbagIO::AddImuHandle(const std::string &topic_name, RosbagIO::ImuHandle f)
    {
        return AddHandle(topic_name, [&f, this](const rosbag::MessageInstance &m) -> bool
                         {
            auto msg = m.template instantiate<sensor_msgs::Imu>();
            if (msg == nullptr) {
                return false;
            }

            IMUPtr imu;
            double scale = 1.0;
            // FIXME:  Livox内置imu的加计需要乘上重力常数,scale=9.885 or scale=1.0
            imu =
                std::make_shared<IMU>(msg->header.stamp.toSec(),
                                      Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                      Vec3d(msg->linear_acceleration.x * scale, msg->linear_acceleration.y * scale,
                                            msg->linear_acceleration.z * scale));

            return f(imu); });
    }

} // namespace zjloc