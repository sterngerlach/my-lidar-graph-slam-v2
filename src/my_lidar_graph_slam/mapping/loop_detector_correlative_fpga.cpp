
/* loop_detector_correlative_fpga.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_correlative_fpga.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorFPGAMetrics::LoopDetectorFPGAMetrics(
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
LoopDetectorCorrelativeFPGA::LoopDetectorCorrelativeFPGA(
    const std::string& loopDetectorName,
    const std::shared_ptr<ScanMatcherCorrelativeFPGA>& scanMatcher,
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
LoopDetectionResultVector LoopDetectorCorrelativeFPGA::Detect(
    const LoopDetectionQueryVector& queries)
{
    LoopDetectionResultVector results;

    /* Create the timer */
    Metric::Timer timer;

    /* Perform loop detection for each query */
    for (auto& query : queries) {
        /* Retrieve the information for each query */
        const auto& scanNode = query.mQueryScanNode;
        const auto& localMap = query.mReferenceLocalMap;
        const auto& localMapNode = query.mReferenceLocalMapNode;

        /* Check the local map Id */
        Assert(localMap.mId == localMapNode.mLocalMapId);
        /* Make sure that the local grid map is in finished state */
        Assert(localMap.mFinished);

        /* Restart the timer */
        timer.Start();

        /* Check that the reference scan node which is closest to the query
         * scan node `scanNode` resides in the reference local grid map */
        const auto& refScanNode = query.mReferenceScanNode;
        Assert(refScanNode.mLocalMapId == localMapNode.mLocalMapId);

        /* Use the pose of the reference scan node `refScanNode` which is
         * closest to the query scan node `scanNode` as the center position
         * of the reference local grid map `localMap` (already obtained as
         * a pose in the coordinate frame local to the `localMap`,
         * so the coordinate trnasformation is not needed) */
        const Point2D<double> localMapCenterPos {
            refScanNode.mLocalPose.mX, refScanNode.mLocalPose.mY };
        /* Compute the scan node pose in a map-local coordinate frame */
        const RobotPose2D<double> mapLocalScanPose = InverseCompound(
            localMapNode.mGlobalPose, scanNode.mGlobalPose);

        /* Use the scan matcher to find the loop */
        const ScanMatchingSummary summary =
            this->mScanMatcher->OptimizePose(
                localMap.mMap, localMap.mId, localMapCenterPos,
                scanNode.mScanData, mapLocalScanPose,
                this->mScoreThreshold, this->mKnownRateThreshold);

        /* Try the next query if the loop is not detected */
        if (!summary.mPoseFound)
            continue;

        /* Refine the loop detection results by performing the scan-matching
         * at the sub-pixel accuracy */
        const ScanMatchingQuery finalQuery {
            localMap.mMap, localMapCenterPos,
            scanNode.mScanData, summary.mEstimatedPose };
        const ScanMatchingSummary finalSummary =
            this->mFinalScanMatcher->OptimizePose(finalQuery);
        /* Check that the scan matching has succeeded */
        Assert(finalSummary.mPoseFound);

        /* Append to the loop detection results */
        results.emplace_back(finalSummary.mEstimatedPose,
                             localMapNode.mGlobalPose,
                             localMapNode.mLocalMapId, scanNode.mNodeId,
                             finalSummary.mEstimatedCovariance);

        /* Update the processing time for loop detections */
        this->mMetrics.mLoopDetectionTime->Observe(timer.ElapsedMicro());
        /* Stop the timer */
        timer.Stop();
    }

    /* Update the metrics */
    this->mMetrics.mNumOfQueries->Observe(queries.size());
    this->mMetrics.mNumOfDetections->Observe(results.size());

    return results;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
