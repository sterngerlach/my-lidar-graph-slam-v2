
/* util.cpp */

#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {

/* Split a given string with a delimiter string */
std::vector<std::string> Split(const std::string& str,
                               const std::string& delimiter)
{
    const auto delimiterLength = delimiter.size();

    if (delimiterLength == 0)
        return std::vector<std::string> { str };

    std::vector<std::string> elementVec;
    auto offsetPos = std::string::size_type(0);

    while (true) {
        const auto foundPos = str.find(delimiter, offsetPos);

        if (foundPos == std::string::npos) {
            elementVec.push_back(str.substr(offsetPos));
            break;
        } else {
            elementVec.push_back(str.substr(offsetPos, foundPos - offsetPos));
            offsetPos = foundPos + delimiterLength;
        }
    }

    return elementVec;
}

/* Split a given string with a delimiter character */
std::vector<std::string> Split(const std::string& str,
                               const char delimiter)
{
    std::vector<std::string> elementVec;
    auto offsetPos = std::string::size_type(0);

    while (true) {
        const auto foundPos = str.find(delimiter, offsetPos);

        if (foundPos == std::string::npos) {
            elementVec.push_back(str.substr(offsetPos));
            break;
        } else {
            elementVec.push_back(str.substr(offsetPos, foundPos - offsetPos));
            offsetPos = foundPos + 1;
        }
    }

    return elementVec;
}

} /* namespace MyLidarGraphSlam */
