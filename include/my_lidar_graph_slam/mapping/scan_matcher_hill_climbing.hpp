
/* scan_matcher_hill_climbing.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HILL_CLIMBING_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HILL_CLIMBING_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct ScanMatcherHillClimbingMetrics
{
    /* Constructor */
    ScanMatcherHillClimbingMetrics(const std::string& scanMatcherName);
    /* Destructor */
    ~ScanMatcherHillClimbingMetrics() = default;

    /* Total processing time for the optimization */
    Metric::DistributionBase*         mOptimizationTime;
    /* Distance between the initial pose and the final pose */
    Metric::DistributionBase*         mDiffTranslation;
    /* Absolute difference between the initial angle and the final angle */
    Metric::DistributionBase*         mDiffRotation;
    /* Total number of the iterations */
    Metric::ValueSequenceBase<int>*   mNumOfIterations;
    /* Total number of the step size updates */
    Metric::ValueSequenceBase<int>*   mNumOfRefinements;
    /* Initial normalized cost value */
    Metric::ValueSequenceBase<float>* mInitialCost;
    /* Final normalized cost value */
    Metric::ValueSequenceBase<float>* mFinalCost;
    /* Number of the scan points in the given scan */
    Metric::ValueSequenceBase<int>*   mNumOfScans;
};

class ScanMatcherHillClimbing final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherHillClimbing(const std::string& scanMatcherName,
                            const double linearStep,
                            const double angularStep,
                            const int maxIterations,
                            const int maxNumOfRefinements,
                            const CostFuncPtr& costFunc);

    /* Destructor */
    ~ScanMatcherHillClimbing() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

private:
    /* Initial step of the linear components (x and y) */
    const double                   mLinearStep;
    /* Initial step of the angular component (theta) */
    const double                   mAngularStep;
    /* Maximum number of iterations */
    const int                      mMaxIterations;
    /* Maximum number of step parameter updates */
    const int                      mMaxNumOfRefinements;
    /* Cost function */
    const CostFuncPtr              mCostFunc;
    /* Metrics information */
    ScanMatcherHillClimbingMetrics mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HILL_CLIMBING_HPP */
