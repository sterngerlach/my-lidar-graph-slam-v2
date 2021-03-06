
/* score_function.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP

#include <cmath>
#include <cstdlib>
#include <memory>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_types.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Declare types for convenience */
class ScoreFunction;
using ScoreFuncPtr = std::shared_ptr<ScoreFunction>;

class ScoreFunction
{
public:
    /*
     * Summary struct holds the details of the result
     */
    struct Summary
    {
        /* Constructor */
        Summary(const double normalizedScore,
                const double score,
                const double knownRate) :
            mNormalizedScore(normalizedScore),
            mScore(score),
            mKnownRate(knownRate) { }

        /* Normalized matching score */
        const double mNormalizedScore;
        /* Matching score */
        const double mScore;
        /* Rate of the grid cells with valid occupancy probability value */
        const double mKnownRate;
    };

public:
    /* Constructor */
    ScoreFunction() = default;
    /* Destructor */
    virtual ~ScoreFunction() = default;

    /* Copy constructor (disabled) */
    ScoreFunction(const ScoreFunction&) = delete;
    /* Copy assignment operator (disabled) */
    ScoreFunction& operator=(const ScoreFunction&) = delete;
    /* Move constructor (disabled) */
    ScoreFunction(ScoreFunction&&) = delete;
    /* Move assignment operator (disabled) */
    ScoreFunction& operator=(ScoreFunction&&) = delete;

    /* Evaluate score function (matching score between scan data and map) */
    virtual Summary Score(const GridMapInterface& gridMap,
                          const Sensor::ScanDataPtr<double>& scanData,
                          const RobotPose2D<double>& mapLocalSensorPose) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP */
