
/* loop_detector_correlative.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_correlative.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <numeric>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorCorrelativeMetrics::LoopDetectorCorrelativeMetrics(
    const std::string& loopDetectorName) :
    mInputSetupTime(nullptr),
    mLoopDetectionTime(nullptr),
    mNumOfQueries(nullptr),
    mNumOfDetections(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the distribution metrics */
    this->mInputSetupTime = pMetricManager->AddDistribution(
        loopDetectorName + ".InputSetupTime");
    this->mLoopDetectionTime = pMetricManager->AddDistribution(
        loopDetectorName + ".LoopDetectionTime");

    /* Register the value sequence metrics */
    this->mNumOfQueries = pMetricManager->AddValueSequenceInt(
        loopDetectorName + ".NumOfQueries");
    this->mNumOfDetections = pMetricManager->AddValueSequenceInt(
        loopDetectorName + ".NumOfDetections");
}

/* Constructor */
LoopDetectorCorrelative::LoopDetectorCorrelative(
    const std::string& loopDetectorName,
    const std::shared_ptr<ScanMatcherCorrelative>& scanMatcher,
    const std::shared_ptr<ScanMatcher>& finalScanMatcher,
    const double scoreThreshold,
    const double knownRateThreshold) :
    LoopDetector(loopDetectorName),
    mScanMatcher(scanMatcher),
    mFinalScanMatcher(finalScanMatcher),
    mScoreThreshold(scoreThreshold),
    mKnownRateThreshold(knownRateThreshold),
    mMetrics(loopDetectorName)
{
    Assert(scoreThreshold > 0.0 && scoreThreshold <= 1.0);
    Assert(knownRateThreshold > 0.0 && knownRateThreshold <= 1.0);
}

/* Find a loop and return a loop constraint */
LoopDetectionResultVector LoopDetectorCorrelative::Detect(
    const LoopDetectionQueryVector& loopDetectionQueries)
{
    LoopDetectionResultVector loopDetectionResults;

    /* Count the number of the successful loop detections */
    int numOfSuccessfulDetections = 0;
    /* Create the timer */
    Metric::Timer timer;

    /* Perform loop detection for each query */
    for (auto& loopDetectionQuery : loopDetectionQueries) {
        /* Retrieve the information for each query */
        const auto& scanNode = loopDetectionQuery.mQueryScanNode;
        const auto& localMap = loopDetectionQuery.mReferenceLocalMap;
        const auto& localMapNode = loopDetectionQuery.mReferenceLocalMapNode;

        /* Check the local map Id */
        Assert(localMap.mId == localMapNode.mLocalMapId);
        /* Make sure that the local grid map is in finished state */
        Assert(localMap.mFinished);

        /* Restart the timer */
        timer.Start();

        /* Precompute a low-resolution grid map */
        if (!this->mPrecompMaps.contains(localMap.mId)) {
            /* Precompute coarser grid maps */
            ConstMapType precompMap =
                this->mScanMatcher->ComputeCoarserMap(localMap.mMap);
            /* Append the newly created grid map */
            this->mPrecompMaps.Append(localMap.mId, std::move(precompMap));
        }

        /* Update the processing time for grid map computations */
        this->mMetrics.mInputSetupTime->Observe(timer.ElapsedMicro());
        /* Restart the timer */
        timer.Start();

        /* Compute the scan node pose in a map-local coordinate frame */
        const RobotPose2D<double> mapLocalScanPose = InverseCompound(
            localMapNode.mGlobalPose, scanNode.mGlobalPose);
        /* Find the corresponding position of the scan node
         * inside the local grid map */
        RobotPose2D<double> correspondingPose;
        Eigen::Matrix3d covarianceMatrix;
        const bool loopDetected = this->FindCorrespondingPose(
            localMap.mMap, this->mPrecompMaps.at(localMap.mId).mMap,
            scanNode.mScanData, mapLocalScanPose,
            correspondingPose, covarianceMatrix);

        /* Do not build a new loop closing edge if loop not detected */
        if (!loopDetected)
            continue;

        /* Check that the reference scan node which is closest to the query
         * scan node `scanNode` resides in the reference local grid map */
        const auto& refScanNode = loopDetectionQuery.mReferenceScanNode;
        Assert(refScanNode.mLocalMapId == localMapNode.mLocalMapId);

        /* Use the pose of the reference scan node `refScanNode` as the
         * center position of the reference local grid map `localMap`,
         * which is represented in the coordinate frame local to `localMap` */
        const auto& localMapCenterPose = refScanNode.mLocalPose;
        const Point2D<double> localMapCenterPos {
            localMapCenterPose.mX, localMapCenterPose.mY };

        /* Refine the loop detection results (relative poses) by performing
         * the scan-matching at sub-pixel accuracy */
        const ScanMatchingQuery finalScanMatchingQuery {
            localMap.mMap, localMapCenterPos,
            scanNode.mScanData, correspondingPose };
        const ScanMatchingSummary finalScanMatchingSummary =
            this->mFinalScanMatcher->OptimizePose(finalScanMatchingQuery);
        /* Make sure that the relative pose estimate is found */
        Assert(finalScanMatchingSummary.mPoseFound);

        /* Append to the loop detection results */
        loopDetectionResults.emplace_back(
            finalScanMatchingSummary.mEstimatedPose,
            localMapNode.mGlobalPose,
            localMapNode.mLocalMapId, scanNode.mNodeId,
            finalScanMatchingSummary.mEstimatedCovariance);
        /* Update the number of the successful loop detections */
        numOfSuccessfulDetections++;

        /* Update the processing time for loop detections */
        this->mMetrics.mLoopDetectionTime->Observe(timer.ElapsedMicro());
        /* Stop the timer */
        timer.Stop();
    }

    /* Update the metrics */
    this->mMetrics.mNumOfQueries->Observe(loopDetectionQueries.size());
    this->mMetrics.mNumOfDetections->Observe(numOfSuccessfulDetections);

    return loopDetectionResults;
}

/* Find a corresponding pose of the current robot pose
 * from a local grid map */
bool LoopDetectorCorrelative::FindCorrespondingPose(
    const GridMapType& localMap,
    const ConstMapType& precompMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalScanPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Just call the scan matcher to find a corresponding pose */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        localMap, precompMap, scanData, mapLocalScanPose,
        this->mScoreThreshold, this->mKnownRateThreshold);

    /* Return the result pose and the covariance in a map-local frame */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    /* Loop detection fails if the score does not exceed the threshold */
    if (!matchingSummary.mPoseFound)
        return false;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
