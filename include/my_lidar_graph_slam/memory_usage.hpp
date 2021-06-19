
/* memory_usage.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MEMORY_USAGE_HPP
#define MY_LIDAR_GRAPH_SLAM_MEMORY_USAGE_HPP

#include <cstdint>

namespace MyLidarGraphSlam {

/* Get the total physical memory usage */
std::uint64_t GetPhysicalMemoryUsage();

/* Get the total virtual memory usage */
std::uint64_t GetVirtualMemoryUsage();

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MEMORY_USAGE_HPP */
