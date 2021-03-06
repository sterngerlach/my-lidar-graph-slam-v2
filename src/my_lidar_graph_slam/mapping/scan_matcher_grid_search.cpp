
/* scan_matcher_branch_bound.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_grid_search.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherGridSearchMetrics::ScanMatcherGridSearchMetrics(
    const std::string& scanMatcherName) :
    mOptimizationTime(nullptr),
    mDiffTranslation(nullptr),
    mDiffRotation(nullptr),
    mNumOfScoreEvaluations(nullptr),
    mNumOfScoreUpdates(nullptr),
    mScoreValue(nullptr),
    mCostValue(nullptr),
    mNumOfScans(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the value sequence metrics */
    this->mOptimizationTime = pMetricManager->AddValueSequence<int>(
        scanMatcherName + ".OptimizationTime");
    this->mDiffTranslation = pMetricManager->AddValueSequence<float>(
        scanMatcherName + ".DiffTranslation");
    this->mDiffRotation = pMetricManager->AddValueSequence<float>(
        scanMatcherName + ".DiffRotation");

    this->mNumOfScoreEvaluations = pMetricManager->AddValueSequence<int>(
        scanMatcherName + ".NumOfScoreEvaluations");
    this->mNumOfScoreUpdates = pMetricManager->AddValueSequence<int>(
        scanMatcherName + ".NumOfScoreUpdates");
    this->mScoreValue = pMetricManager->AddValueSequence<float>(
        scanMatcherName + ".ScoreValue");
    this->mCostValue = pMetricManager->AddValueSequence<float>(
        scanMatcherName + ".CostValue");
    this->mNumOfScans = pMetricManager->AddValueSequence<int>(
        scanMatcherName + ".NumOfScans");
}

/* Constructor */
ScanMatcherGridSearch::ScanMatcherGridSearch(
    const std::string& scanMatcherName,
    const std::shared_ptr<ScorePixelAccurate>& scoreFunc,
    const CostFuncPtr& costFunc,
    const double rangeX,
    const double rangeY,
    const double rangeTheta,
    const double stepX,
    const double stepY,
    const double stepTheta) :
    ScanMatcher(scanMatcherName),
    mScoreFunc(scoreFunc),
    mCostFunc(costFunc),
    mRangeX(rangeX),
    mRangeY(rangeY),
    mRangeTheta(rangeTheta),
    mStepX(stepX),
    mStepY(stepY),
    mStepTheta(stepTheta),
    mMetrics(scanMatcherName)
{
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherGridSearch::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Retrieve the query information */
    const auto& gridMap = queryInfo.mGridMap;
    const auto& scanData = queryInfo.mScanData;
    const RobotPose2D<double>& mapLocalInitialPose =
        queryInfo.mMapLocalInitialPose;

    /* Optimize the robot pose by scan matching */
    return this->OptimizePose(gridMap, scanData,
                              mapLocalInitialPose, 0.0, 0.0);
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherGridSearch::OptimizePose(
    const GridMap& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalInitialPose,
    const double normalizedScoreThreshold,
    const double knownRateThreshold) const
{
    /* Create the timer */
    Metric::Timer timer;

    /* Find the best pose from the search window
     * that maximizes the matching score value */
    const RobotPose2D<double> mapLocalSensorPose =
        Compound(mapLocalInitialPose, scanData->RelativeSensorPose());

    /* Determine the search window radius and step */
    const double rx = this->mRangeX / 2.0;
    const double ry = this->mRangeY / 2.0;
    const double rt = this->mRangeTheta / 2.0;
    const double sx = this->mStepX;
    const double sy = this->mStepY;
    const double st = this->mStepTheta;

    /* Setup the best score */
    double scoreMax = normalizedScoreThreshold;
    /* Setup the number of the score evaluations */
    int numOfScoreEvaluations = 0;
    /* Setup the number of the score updates */
    int numOfScoreUpdates = 0;

    /* Setup the best pose */
    RobotPose2D<double> bestSensorPose = mapLocalSensorPose;

    /* Perform the exhaustive grid search */
    for (double dy = -ry; dy <= ry; dy += sy) {
        for (double dx = -rx; dx <= rx; dx += sx) {
            for (double dt = -rt; dt <= rt; dt += st) {
                /* Calculate the score value */
                const RobotPose2D<double> pose {
                    mapLocalSensorPose.mX + dx,
                    mapLocalSensorPose.mY + dy,
                    mapLocalSensorPose.mTheta + dt };
                const ScoreFunction::Summary scoreSummary =
                    this->mScoreFunc->Score(gridMap, scanData, pose);

                /* Update the number of the score evaluations */
                numOfScoreEvaluations++;

                /* Update the best pose and maximum score value */
                if (scoreSummary.mNormalizedScore > scoreMax &&
                    scoreSummary.mKnownRate > knownRateThreshold) {
                    scoreMax = scoreSummary.mNormalizedScore;
                    bestSensorPose = pose;
                    /* Update the number of the score updates */
                    numOfScoreUpdates++;
                }
            }
        }
    }

    /* The appropriate pose is found if the maximum score is larger than
     * (not larger than or equal to) the score threshold */
    const bool poseFound = scoreMax > normalizedScoreThreshold;

    /* Evaluate the cost value */
    const double costVal = this->mCostFunc->Cost(
        gridMap, scanData, bestSensorPose);
    /* Compute the normalized cost value */
    const double normalizedCost = costVal / scanData->NumOfScans();

    /* Compute the estimated robot pose in a map-local coordinate frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());
    /* Compute the pose covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Update the metrics */
    this->mMetrics.mOptimizationTime->Observe(timer.ElapsedMicro());
    this->mMetrics.mDiffTranslation->Observe(
        Distance(mapLocalInitialPose, estimatedPose));
    this->mMetrics.mDiffRotation->Observe(
        std::abs(mapLocalInitialPose.mTheta - estimatedPose.mTheta));
    this->mMetrics.mNumOfScoreEvaluations->Observe(numOfScoreEvaluations);
    this->mMetrics.mNumOfScoreUpdates->Observe(numOfScoreUpdates);
    this->mMetrics.mScoreValue->Observe(scoreMax);
    this->mMetrics.mCostValue->Observe(normalizedCost);
    this->mMetrics.mNumOfScans->Observe(scanData->NumOfScans());

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated pose covariance matrix in a map-local frame */
    return ScanMatchingSummary {
        poseFound, normalizedCost, mapLocalInitialPose,
        estimatedPose, estimatedCovariance };
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
