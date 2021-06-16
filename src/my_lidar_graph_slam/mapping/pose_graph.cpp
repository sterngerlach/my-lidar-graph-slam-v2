
/* pose_graph.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Inspect the memory usage in bytes */
std::uint64_t PoseGraph::InspectMemoryUsage() const
{
    std::uint64_t memoryUsage = 0;

    memoryUsage += sizeof(this->mLocalMapNodes) +
                   sizeof(this->mScanNodes) +
                   sizeof(this->mEdges) +
                   sizeof(LocalMapNode) * this->mLocalMapNodes.size() +
                   sizeof(ScanNode) * this->mScanNodes.size() +
                   sizeof(PoseGraphEdge) * this->mEdges.size();

    return memoryUsage;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
