
/* loop_detector_branch_bound.cpp */

#include <cassert>
#include <deque>
#include <limits>

#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_branch_bound.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorBranchBoundMetrics::LoopDetectorBranchBoundMetrics(
    const std::string& loopDetectorName) :
    mInputSetupTime(nullptr),
    mLoopDetectionTime(nullptr),
    mNumOfQueries(nullptr),
    mNumOfDetections(nullptr),
    mPrecompMapMemoryUsage(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the value sequence metrics */
    this->mInputSetupTime = pMetricManager->AddValueSequence<int>(
        loopDetectorName + ".InputSetupTime");
    this->mLoopDetectionTime = pMetricManager->AddValueSequence<int>(
        loopDetectorName + ".LoopDetectionTime");
    this->mNumOfQueries = pMetricManager->AddValueSequence<int>(
        loopDetectorName + ".NumOfQueries");
    this->mNumOfDetections = pMetricManager->AddValueSequence<int>(
        loopDetectorName + ".NumOfDetections");
    this->mPrecompMapMemoryUsage =
        pMetricManager->AddValueSequence<std::uint64_t>(
            loopDetectorName + ".PrecompMapMemoryUsage");
}

/* Constructor */
LoopDetectorBranchBound::LoopDetectorBranchBound(
    const std::string& loopDetectorName,
    const std::shared_ptr<ScanMatcherBranchBound>& scanMatcher,
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
LoopDetectionResultVector LoopDetectorBranchBound::Detect(
    const LoopDetectionQueryVector& queries)
{
    LoopDetectionResultVector results;

    /* Create the timer */
    Metric::Timer timer;

    /* Perform loop detection for each query */
    for (auto& query : queries) {
        /* Retrieve the information about each query */
        const auto& scanNode = query.mQueryScanNode;
        const auto& localMap = query.mReferenceLocalMap;
        const auto& localMapNode = query.mReferenceLocalMapNode;

        /* Check the local map Id */
        Assert(localMap.mId == localMapNode.mLocalMapId);
        /* Make sure that the grid map is in finished state */
        Assert(localMap.mFinished);

        /* Restart the timer */
        timer.Start();

        /* Precompute coarser grid maps */
        if (!this->mPrecompMaps.contains(localMap.mId)) {
            /* Precompute multiple coarser grid maps */
            std::vector<ConstMap> precompMaps =
                this->mScanMatcher->ComputeCoarserMaps(localMap.mMap);
            /* Append the newly created grid map pyramid */
            this->mPrecompMaps.Append(localMap.mId, std::move(precompMaps));
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
        const ScanMatchingSummary summary = this->mScanMatcher->OptimizePose(
            localMap.mMap, this->mPrecompMaps.at(localMap.mId).mMaps,
            scanNode.mScanData, mapLocalScanPose,
            this->mScoreThreshold, this->mKnownRateThreshold);

        /* Do not build a new loop closing edge if loop not detected */
        if (!summary.mPoseFound)
            continue;

        /* Check that the reference scan node which is closest to the query
         * scan node `scanNode` resides in the reference local grid map */
        const auto& refScanNode = query.mReferenceScanNode;
        Assert(refScanNode.mLocalMapId == localMapNode.mLocalMapId);

        /* Use the pose of the reference scan node `refScanNode` as the
         * center position of the reference local grid map `localMap`,
         * which is represented in the coordinate frame local to `localMap` */
        const Point2D<double> localMapCenterPos {
            refScanNode.mLocalPose.mX, refScanNode.mLocalPose.mY };

        /* Refine the loop detection results (relative poses) by performing
         * the scan-matching at sub-pixel accuracy */
        const ScanMatchingQuery finalQuery {
            localMap.mMap, localMapCenterPos,
            scanNode.mScanData, summary.mEstimatedPose };
        const ScanMatchingSummary finalSummary =
            this->mFinalScanMatcher->OptimizePose(finalQuery);
        /* Make sure that the relative pose estimate is found */
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

    /* Update the memory usage for the precomputed grid maps */
    using IdDataPair = IdMap<LocalMapId, PrecomputedMapStack>::IdDataPair;
    const std::uint64_t mapMemoryUsage = std::accumulate(
        this->mPrecompMaps.begin(), this->mPrecompMaps.end(), 0,
        [](const std::uint64_t memoryUsage, const IdDataPair& pair) {
            return memoryUsage + pair.mData.InspectMemoryUsage(); });
    this->mMetrics.mPrecompMapMemoryUsage->Observe(mapMemoryUsage);

    return results;
}

} /* Mapping */
} /* namespace MyLidarGraphSlam */
