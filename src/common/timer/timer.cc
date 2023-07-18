#include "timer.h"

#include <glog/logging.h>
#include <fstream>
#include <numeric>

namespace zjloc::common
{

    std::map<std::string, Timer::TimerRecord> Timer::records_;

    void Timer::PrintAll()
    {
        std::cout << ANSI_COLOR_GREEN_BOLD << ">>> ===== Printing run time =====" << std::endl;
        for (const auto &r : records_)
        {
            std::cout << ANSI_COLOR_GREEN << "> [ " << r.first << " ] average time usage: "
                      << std::accumulate(r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), 0.0) /
                             double(r.second.time_usage_in_ms_.size())
                      << " ms , called times: " << r.second.time_usage_in_ms_.size() << std::endl;
        }
        std::cout << ">>> ===== Printing run time end =====" << ANSI_COLOR_RESET << std::endl;
    }

    void Timer::DumpIntoFile(const std::string &file_name)
    {
        std::ofstream ofs(file_name, std::ios::out);
        if (!ofs.is_open())
        {
            std::cout << ANSI_COLOR_RED_BOLD << "Failed to open file: " << file_name << std::endl;
            return;
        }
        else
        {
            std::cout << ANSI_COLOR_GREEN_BOLD << "Dump Time Records into file: " << file_name << ANSI_COLOR_RESET << std::endl;
        }

        size_t max_length = 0;
        for (const auto &iter : records_)
        {
            ofs << iter.first << ", ";
            if (iter.second.time_usage_in_ms_.size() > max_length)
            {
                max_length = iter.second.time_usage_in_ms_.size();
            }
        }
        ofs << std::endl;

        for (size_t i = 0; i < max_length; ++i)
        {
            for (const auto &iter : records_)
            {
                if (i < iter.second.time_usage_in_ms_.size())
                {
                    ofs << iter.second.time_usage_in_ms_[i] << ",";
                }
                else
                {
                    ofs << ",";
                }
            }
            ofs << std::endl;
        }
        ofs.close();
    }

    double Timer::GetMeanTime(const std::string &func_name)
    {
        if (records_.find(func_name) == records_.end())
        {
            return 0.0;
        }

        auto r = records_[func_name];
        return std::accumulate(r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0) /
               double(r.time_usage_in_ms_.size());
    }

} // namespace zjloc::utils