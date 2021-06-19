
/* memory_usage.cpp */

#include "my_lidar_graph_slam/memory_usage.hpp"

#include <algorithm>
#include <fstream>
#include <string>

namespace MyLidarGraphSlam {

/* Parse the line in `/proc/self/status` file */
std::uint64_t ParseProcStatus(const std::string& line)
{
    const auto isDigit = [](const char c) { return c >= '0' && c <= '9'; };
    const auto digitIt = std::find_if(line.begin(), line.end(), isDigit);
    const auto endIt = std::find_if_not(digitIt, line.end(), isDigit);

    if (digitIt == line.end())
        return 0;

    const auto digitIdx = std::distance(line.begin(), digitIt);
    const auto digitLen = std::distance(digitIt, endIt);
    const std::string digitStr = line.substr(digitIdx, digitLen);
    const std::uint64_t value = std::stoull(digitStr);

    return value;
}

/* Read the `/proc/self/status` and read the entry */
std::uint64_t ReadProcStatus(const std::string& entryName)
{
    std::ifstream procStatusFile { "/proc/self/status" };

    if (!procStatusFile)
        return 0;

    std::string line;
    std::uint64_t value = 0;

    /* Find and read the entry */
    while (std::getline(procStatusFile, line)) {
        if (line.size() >= entryName.size() &&
            std::equal(entryName.begin(), entryName.end(), line.begin())) {
            value = ParseProcStatus(line);
            break;
        }
    }

    procStatusFile.close();

    return value;
}

/* Get the total physical memory usage of this process in bytes */
std::uint64_t GetPhysicalMemoryUsage()
{
    return ReadProcStatus("VmRSS:") * (1 << 10);
}

/* Get the total virtual memory usage of this process in bytes */
std::uint64_t GetVirtualMemoryUsage()
{
    return ReadProcStatus("VmSize:") * (1 << 10);
}

} /* namespace MyLidarGraphSlam */
