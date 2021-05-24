
/* cost_function_greedy_endpoint.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class CostGreedyEndpoint;
using CostGreedyEndpointPtr = std::shared_ptr<CostGreedyEndpoint>;

class CostGreedyEndpoint final : public CostFunction
{
public:
    /* Constructor */
    CostGreedyEndpoint(const double mapResolution,
                       const double hitAndMissedDist,
                       const double occupancyThreshold,
                       const int kernelSize,
                       const double scalingFactor,
                       const double standardDeviation);

    /* Destructor */
    ~CostGreedyEndpoint() = default;

    /* Calculate cost function (mismatch between scan data and map) */
    double Cost(
        const GridMapInterfaceType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a gradient vector in a map-local coordinate frame */
    Eigen::Vector3d ComputeGradient(
        const GridMapInterfaceType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a covariance matrix in a map-local coordinate frame */
    Eigen::Matrix3d ComputeCovariance(
        const GridMapInterfaceType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

private:
    /* Setup the lookup table for the cost value */
    void SetupLookupTable();

private:
    /* Grid map resolution (in meters) */
    const double              mMapResolution;
    /* Distance between hit and missed grid cells */
    const double              mHitAndMissedDist;
    /* Probability threshold for being obstructed */
    const double              mOccupancyThreshold;
    /* Size of searching window (in the number of grid cells) */
    const int                 mKernelSize;
    /* Standard deviation of the Gaussian distribution of the error */
    const double              mStandardDeviation;
    /* Variance of the Gaussian distribution of the error */
    const double              mVariance;
    /* Scaling factor for cost value */
    const double              mScalingFactor;
    /* Lookup table for the cost value */
    std::unique_ptr<double[]> mCostLookupTable;
    /* Maximum cost value */
    double                    mDefaultCostValue;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP */
