
/* score_function_pixel_accurate.cpp */

#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScorePixelAccurate::ScorePixelAccurate() :
    ScoreFunction()
{
}

/* Evaluate score function (matching score between scan data and map) */
ScoreFunction::Summary ScorePixelAccurate::Score(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    double sumScore = 0.0;

    const std::size_t numOfScans = scanData->NumOfScans();
    std::size_t numOfKnownGridCells = 0;

    const double unknownProb = gridMap.UnknownProbability();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Add the occupancy probability value at the hit point */
        const Point2D<double> localHitPoint =
            scanData->HitPoint(mapLocalSensorPose, i);
        const Point2D<int> hitPointIdx =
            gridMap.PositionToIndex(localHitPoint.mX, localHitPoint.mY);
        const double hitCellProb = gridMap.ProbabilityOr(
            hitPointIdx.mY, hitPointIdx.mX, unknownProb);

        /* Ignore the grid cell with unknown occupancy probability */
        if (hitCellProb == unknownProb)
            continue;

        /* If the hit grid cell has unknown probability value,
         * the minimum score (zero) is added */
        sumScore += hitCellProb;
        ++numOfKnownGridCells;
    }

    /* Normalize the score function */
    const double normalizedScore =
        sumScore / static_cast<double>(scanData->NumOfScans());

    /* Calculate the rate of valid grid cells */
    const double knownRate =
        static_cast<double>(numOfKnownGridCells) /
        static_cast<double>(numOfScans);

    /* Return the score evaluation summary */
    return Summary { normalizedScore, sumScore, knownRate };
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
