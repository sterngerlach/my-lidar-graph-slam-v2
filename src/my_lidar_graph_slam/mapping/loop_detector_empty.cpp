
/* loop_detector_empty.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_empty.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Do not perform loop detection */
LoopDetectionResultVector LoopDetectorEmpty::Detect(
    const LoopDetectionQueryVector& /* loopDetectionQueries */)
{
    /* Do not perform loop detection */
    /* Clear the loop detection results */
    /* Empty result indicates that the loop detection failed */
    return LoopDetectionResultVector { };
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
