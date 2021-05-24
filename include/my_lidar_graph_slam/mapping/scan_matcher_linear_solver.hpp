
/* scan_matcher_linear_solver.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_LINEAR_SOLVER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_LINEAR_SOLVER_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_square_error.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct ScanMatcherLinearSolverMetrics
{
    /* Constructor */
    ScanMatcherLinearSolverMetrics(const std::string& scanMatcherName);
    /* Destructor */
    ~ScanMatcherLinearSolverMetrics() = default;

    /* Total processing time for the optimization */
    Metric::DistributionBase*  mOptimizationTime;
    /* Distance between the initial pose and the final pose */
    Metric::DistributionBase*  mDiffTranslation;
    /* Absolute difference between the initial angle and the final angle */
    Metric::DistributionBase*  mDiffRotation;
    /* Total number of the iterations */
    Metric::ValueSequenceBase* mNumOfIterations;
    /* Initial normalized cost value */
    Metric::ValueSequenceBase* mInitialCost;
    /* Final normalized cost value */
    Metric::ValueSequenceBase* mFinalCost;
    /* Number of the scan points in the given scan */
    Metric::ValueSequenceBase* mNumOfScans;
};

class ScanMatcherLinearSolver final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherLinearSolver(const std::string& scanMatcherName,
                            const int numOfIterationsMax,
                            const double convergenceThreshold,
                            const double initialLambda,
                            const CostFuncPtr& costFunc);

    /* Destructor */
    ~ScanMatcherLinearSolver() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

private:
    /* Perform one optimization step */
    RobotPose2D<double> OptimizeStep(
        const GridMapType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose);

private:
    /* Maximum number of the optimization iterations */
    const int                      mNumOfIterationsMax;
    /* Threshold to check the convergence */
    const double                   mConvergenceThreshold;
    /* Damping factor in Levenberg-Marquardt method */
    double                         mLambda;
    /* Cost function */
    CostSquareErrorPtr             mCostFunc;
    /* Metrics information */
    ScanMatcherLinearSolverMetrics mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_LINEAR_SOLVER_HPP */
