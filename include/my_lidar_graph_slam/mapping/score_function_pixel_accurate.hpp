
/* score_function_pixel_accurate.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP

#include "my_lidar_graph_slam/mapping/score_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScorePixelAccurate final : public ScoreFunction
{
public:
    /* Constructor */
    ScorePixelAccurate();
    /* Destructor */
    ~ScorePixelAccurate() = default;

    /* Evaluate score function (matching score between scan data and map) */
    Summary Score(const GridMapInterface& gridMap,
                  const Sensor::ScanDataPtr<double>& scanData,
                  const RobotPose2D<double>& mapLocalSensorPose) override;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP */
