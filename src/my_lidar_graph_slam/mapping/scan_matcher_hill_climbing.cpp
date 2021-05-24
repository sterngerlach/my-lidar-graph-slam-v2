
/* scan_matcher_hill_climbing.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_hill_climbing.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherHillClimbingMetrics::ScanMatcherHillClimbingMetrics(
    const std::string& scanMatcherName) :
    mOptimizationTime(nullptr),
    mDiffTranslation(nullptr),
    mDiffRotation(nullptr),
    mNumOfIterations(nullptr),
    mNumOfRefinements(nullptr),
    mInitialCost(nullptr),
    mFinalCost(nullptr),
    mNumOfScans(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the distribution metrics */
    this->mOptimizationTime = pMetricManager->AddDistribution(
        scanMatcherName + ".OptimizationTime");
    this->mDiffTranslation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffTranslation");
    this->mDiffRotation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffRotation");

    /* Register the value sequence metrics */
    this->mNumOfIterations = pMetricManager->AddValueSequenceInt(
        scanMatcherName + ".NumOfIterations");
    this->mNumOfRefinements = pMetricManager->AddValueSequenceInt(
        scanMatcherName + ".NumOfRefinements");
    this->mInitialCost = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".InitialCost");
    this->mFinalCost = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".FinalCost");
    this->mNumOfScans = pMetricManager->AddValueSequenceInt(
        scanMatcherName + ".NumOfScans");
}

/* Constructor */
ScanMatcherHillClimbing::ScanMatcherHillClimbing(
    const std::string& scanMatcherName,
    const double linearStep,
    const double angularStep,
    const int maxIterations,
    const int maxNumOfRefinements,
    const CostFuncPtr& costFunc) :
    ScanMatcher(scanMatcherName),
    mLinearStep(linearStep),
    mAngularStep(angularStep),
    mMaxIterations(maxIterations),
    mMaxNumOfRefinements(maxNumOfRefinements),
    mCostFunc(costFunc),
    mMetrics(scanMatcherName)
{
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherHillClimbing::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Create the timer */
    Metric::Timer timer;

    /* Constants used for hill-climbing method */
    static const double moveX[] = { 1.0, -1.0, 0.0, 0.0, 0.0, 0.0 };
    static const double moveY[] = { 0.0, 0.0, 1.0, -1.0, 0.0, 0.0 };
    static const double moveTheta[] = { 0.0, 0.0, 0.0, 0.0, 1.0, -1.0 };

    /* Retrieve the query information */
    const auto& gridMap = queryInfo.mGridMap;
    const auto& scanData = queryInfo.mScanData;
    const RobotPose2D<double>& mapLocalInitialPose =
        queryInfo.mMapLocalInitialPose;

    /* Calculate the sensor pose from the initial robot pose
     * Sensor pose is in the map-local coordinate frame */
    const RobotPose2D<double>& relPose = scanData->RelativeSensorPose();
    const RobotPose2D<double> mapLocalSensorPose =
        Compound(mapLocalInitialPose, relPose);

    /* Compute the initial cost value */
    const double initialCost = this->mCostFunc->Cost(
        gridMap, scanData, mapLocalSensorPose);
    const double normalizedInitialCost = initialCost / scanData->NumOfScans();

    /* Minimum cost and the best pose in the map-local coordinate frame */
    double minCost = initialCost;
    RobotPose2D<double> bestSensorPose = mapLocalSensorPose;

    /* The number of iterations */
    int numOfIterations = 0;
    /* The number of refinements (step parameter updates) */
    int numOfRefinements = 0;

    double currentLinearStep = this->mLinearStep;
    double currentAngularStep = this->mAngularStep;
    bool poseUpdated = false;

    do {
        /* Local optimization */
        double minLocalCost = minCost;
        RobotPose2D<double> bestLocalPose = bestSensorPose;
        poseUpdated = false;

        for (int i = 0; i < 6; ++i) {
            /* Move forward, backward, left, right,
             * rotate left and rotate right a little bit
             * then calculate the cost value */
            RobotPose2D<double> localPose = bestSensorPose;
            localPose.mX += moveX[i] * currentLinearStep;
            localPose.mY += moveY[i] * currentLinearStep;
            localPose.mTheta += moveTheta[i] * currentAngularStep;

            /* Calculate the cost */
            const double localCost =
                this->mCostFunc->Cost(gridMap, scanData, localPose);

            if (localCost < minLocalCost) {
                minLocalCost = localCost;
                bestLocalPose = localPose;
                poseUpdated = true;
            }
        }

        /* Update best pose */
        if (poseUpdated) {
            minCost = minLocalCost;
            bestSensorPose = bestLocalPose;
        } else {
            /* Update the step value if pose not improved */
            ++numOfRefinements;
            currentLinearStep *= 0.5;
            currentAngularStep *= 0.5;
        }
    } while ((poseUpdated || numOfRefinements < this->mMaxNumOfRefinements) &&
             (++numOfIterations < this->mMaxIterations));

    /* Compute the normalized cost value */
    const double normalizedCost = minCost / scanData->NumOfScans();
    /* Compute the estimated robot pose in a map-local coordinate frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestSensorPose, relPose);
    /* Calculate the pose covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Update the metrics */
    this->mMetrics.mOptimizationTime->Observe(timer.ElapsedMicro());
    this->mMetrics.mDiffTranslation->Observe(
        Distance(mapLocalInitialPose, estimatedPose));
    this->mMetrics.mDiffRotation->Observe(
        std::abs(mapLocalInitialPose.mTheta - estimatedPose.mTheta));
    this->mMetrics.mNumOfIterations->Observe(numOfIterations);
    this->mMetrics.mNumOfRefinements->Observe(numOfRefinements);
    this->mMetrics.mInitialCost->Observe(normalizedInitialCost);
    this->mMetrics.mFinalCost->Observe(normalizedCost);
    this->mMetrics.mNumOfScans->Observe(scanData->NumOfScans());

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated covariance matrix in a map-local frame */
    return ScanMatchingSummary {
        true, normalizedCost, mapLocalInitialPose,
        estimatedPose, estimatedCovariance };
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
