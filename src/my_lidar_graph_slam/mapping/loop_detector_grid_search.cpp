
/* loop_detector_grid_search.cpp */

#include <cassert>
#include <limits>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_grid_search.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorGridSearchMetrics::LoopDetectorGridSearchMetrics(
    const std::string& loopDetectorName) :
    mLoopDetectionTime(nullptr),
    mNumOfQueries(nullptr),
    mNumOfDetections(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the value sequence metrics */
    this->mLoopDetectionTime = pMetricManager->AddValueSequence<int>(
        loopDetectorName + ".LoopDetectionTime");
    this->mNumOfQueries = pMetricManager->AddValueSequence<int>(
        loopDetectorName + ".NumOfQueries");
    this->mNumOfDetections = pMetricManager->AddValueSequence<int>(
        loopDetectorName + ".NumOfDetections");
}

/* Constructor */
LoopDetectorGridSearch::LoopDetectorGridSearch(
    const std::string& loopDetectorName,
    const std::shared_ptr<ScanMatcherGridSearch>& scanMatcher,
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
LoopDetectionResultVector LoopDetectorGridSearch::Detect(
    const LoopDetectionQueryVector& loopDetectionQueries)
{
    LoopDetectionResultVector loopDetectionResults;

    /* Count the number of the successful loop detections */
    int numOfSuccessfulDetections = 0;
    /* Create the timer */
    Metric::Timer timer;

    /* Perform loop detection for each query */
    for (const auto& loopDetectionQuery : loopDetectionQueries) {
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

        /* Compute the scan node pose in a map-local coordinate frame */
        const RobotPose2D<double> mapLocalScanPose = InverseCompound(
            localMapNode.mGlobalPose, scanNode.mGlobalPose);
        /* Find the corresponding position of the scan node
         * inside the local grid map */
        RobotPose2D<double> correspondingPose;
        Eigen::Matrix3d covarianceMatrix;
        const bool loopDetected = this->FindCorrespondingPose(
            localMap.mMap, scanNode.mScanData, mapLocalScanPose,
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
        /* Relative pose and covariance matrix in a map-local coordinate
         * frame is already obtained */
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
 * from the local grid map */
bool LoopDetectorGridSearch::FindCorrespondingPose(
    const GridMap& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalScanPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Just call the exhaustive grid search scan matcher */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        gridMap, scanData, mapLocalScanPose,
        this->mScoreThreshold, this->mKnownRateThreshold);

    /* Return the result */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    /* Loop detection fails if the matching score falls below the threshold */
    if (!matchingSummary.mPoseFound)
        return false;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
