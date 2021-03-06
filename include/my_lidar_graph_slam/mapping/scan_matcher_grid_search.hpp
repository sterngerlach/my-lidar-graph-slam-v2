
/* scan_matcher_grid_search.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GRID_SEARCH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GRID_SEARCH_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct ScanMatcherGridSearchMetrics
{
    /* Constructor */
    ScanMatcherGridSearchMetrics(const std::string& scanMatcherName);
    /* Destructor */
    ~ScanMatcherGridSearchMetrics() = default;

    /* Total processing time for the optimization */
    Metric::ValueSequenceBase<int>*   mOptimizationTime;
    /* Distance between the initial pose and the final pose */
    Metric::ValueSequenceBase<float>* mDiffTranslation;
    /* Absolute difference between the initial angle and the final angle */
    Metric::ValueSequenceBase<float>* mDiffRotation;
    /* Total number of the score evaluations */
    Metric::ValueSequenceBase<int>*   mNumOfScoreEvaluations;
    /* Total number of the score updates */
    Metric::ValueSequenceBase<int>*   mNumOfScoreUpdates;
    /* Normalized score value of the best solution */
    Metric::ValueSequenceBase<float>* mScoreValue;
    /* Normalized cost value of the best solution */
    Metric::ValueSequenceBase<float>* mCostValue;
    /* Number of the scan points in the given scan */
    Metric::ValueSequenceBase<int>*   mNumOfScans;
};

class ScanMatcherGridSearch final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherGridSearch(
        const std::string& scanMatcherName,
        const std::shared_ptr<ScorePixelAccurate>& scoreFunc,
        const CostFuncPtr& costFunc,
        const double rangeX,
        const double rangeY,
        const double rangeTheta,
        const double stepX,
        const double stepY,
        const double stepTheta);

    /* Destructor */
    ~ScanMatcherGridSearch() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const GridMap& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalInitialPose,
        const double normalizedScoreThreshold,
        const double knownRateThreshold) const;

private:
    /* Pixel-accurate score function */
    std::shared_ptr<ScorePixelAccurate> mScoreFunc;
    /* Cost function */
    CostFuncPtr                         mCostFunc;
    /* Linear (horizontal) size of the search window */
    const double                        mRangeX;
    /* Linear (vertical) size of the search window */
    const double                        mRangeY;
    /* Angular size of the search window */
    const double                        mRangeTheta;
    /* Linear (horizontal) step size */
    const double                        mStepX;
    /* Linear (vertical) step size */
    const double                        mStepY;
    /* Angular step size */
    const double                        mStepTheta;
    /* Metrics information */
    ScanMatcherGridSearchMetrics        mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GRID_SEARCH_HPP */
