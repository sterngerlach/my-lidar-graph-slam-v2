
/* scan_matcher_correlative.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_CORRELATIVE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_CORRELATIVE_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct ScanMatcherCorrelativeMetrics
{
    /* Constructor */
    ScanMatcherCorrelativeMetrics(const std::string& scanMatcherName);
    /* Destructor */
    ~ScanMatcherCorrelativeMetrics() = default;

    /* Total processing time for setting up the input */
    Metric::DistributionBase*         mInputSetupTime;
    /* Total processing time for the optimization */
    Metric::DistributionBase*         mOptimizationTime;
    /* Distance between the initial pose and the final pose */
    Metric::DistributionBase*         mDiffTranslation;
    /* Absolute difference between the initial angle and the final angle */
    Metric::DistributionBase*         mDiffRotation;
    /* Size of the search window along the X-axis */
    Metric::DistributionBase*         mWinSizeX;
    /* Size of the search window along the Y-axis */
    Metric::DistributionBase*         mWinSizeY;
    /* Size of the search window along the Theta-axis */
    Metric::DistributionBase*         mWinSizeTheta;
    /* Step size along the X-axis */
    Metric::DistributionBase*         mStepSizeX;
    /* Step size along the Y-axis */
    Metric::DistributionBase*         mStepSizeY;
    /* Step size along the Theta-axis */
    Metric::DistributionBase*         mStepSizeTheta;
    /* Total number of the skipped nodes */
    Metric::CounterBase*              mNumOfIgnoredNodes;
    /* Total number of the processed nodes */
    Metric::CounterBase*              mNumOfProcessedNodes;
    /* Normalized score value of the best solution */
    Metric::ValueSequenceBase<float>* mScoreValue;
    /* Normalized cost value of the best solution */
    Metric::ValueSequenceBase<float>* mCostValue;
    /* Number of the scan points in the given scan */
    Metric::ValueSequenceBase<int>*   mNumOfScans;
};

class ScanMatcherCorrelative final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherCorrelative(
        const std::string& scanMatcherName,
        const CostFuncPtr& costFunc,
        const int lowResolution,
        const double rangeX,
        const double rangeY,
        const double rangeTheta);

    /* Destructor */
    ~ScanMatcherCorrelative() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const GridMap& gridMap,
        const ConstMap& precompMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalInitialPose,
        const double normalizedScoreThreshold,
        const double knownRateThreshold) const;

    /* Precompute a coarser grid map for scan matching */
    ConstMap ComputeCoarserMap(const GridMap& gridMap) const;

private:
    /* Compute the search step */
    void ComputeSearchStep(const GridMapInterface& gridMap,
                           const Sensor::ScanDataPtr<double>& scanData,
                           double& stepX,
                           double& stepY,
                           double& stepTheta) const;

    /* Compute the grid cell indices for scan points */
    void ComputeScanIndices(
        const ConstMap& precompMap,
        const RobotPose2D<double>& mapLocalSensorPose,
        const Sensor::ScanDataPtr<double>& scanData,
        std::vector<Point2D<int>>& scanIndices) const;

    /* Compute the scan matching score based on the already projected
     * scan points (indices) and index offsets */
    ScoreFunction::Summary ComputeScore(
        const GridMapInterface& gridMap,
        const std::vector<Point2D<int>>& scanIndices,
        const int offsetX,
        const int offsetY) const;

    /* Evaluate the matching score using high-resolution grid map */
    void EvaluateHighResolutionMap(
        const GridMapInterface& gridMap,
        const std::vector<Point2D<int>>& scanIndices,
        const int offsetX,
        const int offsetY,
        const int offsetTheta,
        int& maxWinX,
        int& maxWinY,
        int& maxWinTheta,
        double& maxScore) const;

private:
    /* Cost function just for calculating the pose covariance matrix */
    const CostFuncPtr             mCostFunc;
    /* Resolution for low resolution map (in the number of grid cells) */
    const int                     mLowResolution;
    /* Linear (horizontal) size of the searching window */
    const double                  mRangeX;
    /* Linear (vertical) size of the searching window */
    const double                  mRangeY;
    /* Angular range of the searching window */
    const double                  mRangeTheta;
    /* Metrics information */
    ScanMatcherCorrelativeMetrics mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_CORRELATIVE_HPP */
