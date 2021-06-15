
/* loop_detector_fpga_parallel.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_fpga_parallel.hpp"

#include <thread>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorFPGAParallel::LoopDetectorFPGAParallel(
    const std::string& loopDetectorName,
    const std::shared_ptr<ScanMatcherCorrelativeFPGA>& scanMatcher0,
    const std::shared_ptr<ScanMatcherCorrelativeFPGA>& scanMatcher1,
    const std::shared_ptr<ScanMatcher>& finalScanMatcher0,
    const std::shared_ptr<ScanMatcher>& finalScanMatcher1,
    const double scoreThreshold,
    const double knownRateThreshold) :
    LoopDetector(loopDetectorName),
    mScanMatcher({ scanMatcher0, scanMatcher1 }),
    mFinalScanMatcher({ finalScanMatcher0, finalScanMatcher1 }),
    mScoreThreshold(scoreThreshold),
    mKnownRateThreshold(knownRateThreshold),
    mMetrics(loopDetectorName)
{
    Assert(scoreThreshold > 0.0 && scoreThreshold <= 1.0);
    Assert(knownRateThreshold > 0.0 && knownRateThreshold <= 1.0);
}

/* Find a loop and return a loop constraint */
LoopDetectionResultVector LoopDetectorFPGAParallel::Detect(
    const LoopDetectionQueryVector& queries)
{
    /* Perform the loop detections */
    LoopDetectionResultVector results;
    LoopDetectionResultVector results1;

    /* Start a sub-thread to process the second half of the queries */
    auto subThread = std::thread([&]() {
        this->Detect(1, queries.size() / 2, queries.size(),
                     queries, results1); });
    /* Start to process the first half of the queries */
    this->Detect(0, 0, queries.size() / 2, queries, results);

    /* Wait for the sub-thread to finish the job */
    if (subThread.joinable())
        subThread.join();

    /* Concatenate the results */
    for (const auto& result : results1)
        results.emplace_back(result.mRelativePose, result.mLocalMapPose,
                             result.mLocalMapNodeId, result.mScanNodeId,
                             result.mEstimatedCovMat);

    /* Update the metrics */
    this->mMetrics.mNumOfQueries->Observe(queries.size());
    this->mMetrics.mNumOfDetections->Observe(results.size());

    return results;
}

/* Perform a loop detection given a query */
void LoopDetectorFPGAParallel::Detect(
    const int coreId,
    const std::size_t queryBeginIdx,
    const std::size_t queryEndIdx,
    const LoopDetectionQueryVector& queries,
    LoopDetectionResultVector& results)
{
    /* Create the timer */
    Metric::Timer timer;

    for (std::size_t i = queryBeginIdx; i < queryEndIdx; ++i) {
        /* Get the query information */
        const auto& scanNode = queries[i].mQueryScanNode;
        const auto& localMap = queries[i].mReferenceLocalMap;
        const auto& localMapNode = queries[i].mReferenceLocalMapNode;

        /* Check the local map Id */
        Assert(localMap.mId == localMapNode.mLocalMapId);
        /* Make sure that the local grid map is in finished state */
        Assert(localMap.mFinished);

        /* Restart the timer */
        timer.Start();

        /* Check that the reference scan node, which is closest to the query
         * scan node `scanNode` resides in the reference local grid map */
        const auto& refScanNode = queries[i].mReferenceScanNode;
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
            this->mScanMatcher[coreId]->OptimizePose(
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
            this->mFinalScanMatcher[coreId]->OptimizePose(finalQuery);
        /* Check that the scan matching has succeeded */
        Assert(finalSummary.mPoseFound);

        /* Append to the loop detection results */
        results.emplace_back(finalSummary.mEstimatedPose,
                             localMapNode.mGlobalPose,
                             localMapNode.mLocalMapId, scanNode.mNodeId,
                             finalSummary.mEstimatedCovariance);

        /* Measure the processing time for the loop detection */
        this->mMetrics.mLoopDetectionTime->Observe(timer.ElapsedMicro());
        /* Stop the timer */
        timer.Stop();
    }
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
