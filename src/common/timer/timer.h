#ifndef FUSION_TIMER_H
#define FUSION_TIMER_H

#include <ctime>
#include <iostream>
#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <cstdlib>
#include "tools/tool_color_printf.hpp"

namespace zjloc::common
{

    /// 统计时间工具
    /// NOTE Timer套在gtest当中貌似有问题
    class Timer
    {
    public:
        struct TimerRecord
        {
            TimerRecord() = default;
            TimerRecord(const std::string &name, double time_usage)
            {
                func_name_ = name;
                time_usage_in_ms_.emplace_back(time_usage);
            }
            std::string func_name_;
            std::vector<double> time_usage_in_ms_;
        };

        /**
         * 评价并记录函数用时
         * @tparam F
         * @param func
         * @param func_name
         */
        template <class F>
        static void Evaluate(F &&func, const std::string &func_name)
        {
            auto t1 = std::chrono::steady_clock::now();
            std::forward<F>(func)();
            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

            if (records_.find(func_name) != records_.end())
            {
                records_[func_name].time_usage_in_ms_.emplace_back(time_used);
            }
            else
            {
                records_.insert({func_name, TimerRecord(func_name, time_used)});
            }
        }

        /// 打印记录的所有耗时
        static void PrintAll();

        /// 写入文件，方便作图分析
        static void DumpIntoFile(const std::string &file_name);

        /// 获取某个函数的平均执行时间
        static double GetMeanTime(const std::string &func_name);

        /// 清理记录
        static void Clear() { records_.clear(); }

    private:
        static std::map<std::string, TimerRecord> records_;
    };

    class TicToc
    {
    public:
        TicToc()
        {
            tic();
        }

        TicToc(bool _disp)
        {
            disp_ = _disp;
            tic();
        }

        void tic()
        {
            start = std::chrono::system_clock::now();
        }

        void toc(std::string _about_task)
        {
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            double elapsed_ms = elapsed_seconds.count() * 1000;

            if (disp_)
            {
                std::cout.precision(3); // 10 for sec, 3 for ms
                std::cout << _about_task << ": " << elapsed_ms << " msec." << std::endl;
            }
        }
        double toc()
        {
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            double elapsed_ms = elapsed_seconds.count() * 1000;
            return elapsed_ms;
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> start, end;
        bool disp_ = false;
    };

} // namespace zjloc::utils

#endif // FUSION_TIMER_H
