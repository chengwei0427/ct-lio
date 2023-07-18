//
// Created by xiang on 2021/7/20.
//

#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <functional>
#include <utility>

#include "common/global_flags.h"
#include "tools/imu.h"
#include "common/lidar_utils.h"
#include "common/math_utils.h"
#include "tools/message_def.h"
#include "tools/odom.h"

namespace zjloc
{

    /**
     * 读取本书提供的数据文本文件，并调用回调函数
     * 数据文本文件主要提供IMU/Odom读数
     */
    class TxtIO
    {
    public:
        TxtIO(const std::string &file_path) : fin(file_path) {}

        /// 定义回调函数
        using IMUProcessFuncType = std::function<void(const IMU &)>;
        using OdomProcessFuncType = std::function<void(const Odom &)>;

        TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc)
        {
            imu_proc_ = std::move(imu_proc);
            return *this;
        }

        TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc)
        {
            odom_proc_ = std::move(odom_proc);
            return *this;
        }

        // 遍历文件内容，调用回调函数
        void Go();

    private:
        std::ifstream fin;
        IMUProcessFuncType imu_proc_;
        OdomProcessFuncType odom_proc_;
    };

    /**
     * ROSBAG IO
     * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
     */
    class RosbagIO
    {
    public:
        explicit RosbagIO(std::string bag_file) : bag_file_(std::move(bag_file)) {}

        using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance &m)>;

        /// 一些方便直接使用的topics, messages
        using Scan2DHandle = std::function<bool(sensor_msgs::LaserScanPtr)>;
        using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2::Ptr)>;
        using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
        using ImuHandle = std::function<bool(IMUPtr)>;
        using OdomHandle = std::function<bool(const Odom &)>;

        // 遍历文件内容，调用回调函数
        void Go();

        /// 通用处理函数
        RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func)
        {
            process_func_.emplace(topic_name, func);
            return *this;
        }

        /// 2D激光处理
        RosbagIO &AddScan2DHandle(const std::string &topic_name, Scan2DHandle f)
        {
            return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool
                             {
            auto msg = m.instantiate<sensor_msgs::LaserScan>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg); });
        }

        /// point cloud2 的处理
        RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f)
        {
            return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool
                             {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg); });
        }

        /// IMU
        RosbagIO &AddImuHandle(const std::string &imu_topic, RosbagIO::ImuHandle f);

        /// 清除现有的处理函数
        void CleanProcessFunc() { process_func_.clear(); }

    private:
        std::map<std::string, MessageProcessFunction> process_func_;
        std::string bag_file_;
    };

} // namespace zjloc

#endif // SLAM_IN_AUTO_DRIVING_IO_UTILS_H
