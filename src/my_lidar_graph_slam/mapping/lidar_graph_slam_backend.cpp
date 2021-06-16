
/* lidar_graph_slam_backend.cpp */

#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
BackendMetrics::BackendMetrics() :
    mProcessTime(nullptr),
    mLoopSearchSetupTime(nullptr),
    mLoopSearchTime(nullptr),
    mLoopDetectionSetupTime(nullptr),
    mLoopDetectionTime(nullptr),
    mPoseGraphAppendTime(nullptr),
    mOptimizationSetupTime(nullptr),
    mOptimizationTime(nullptr),
    mPoseGraphUpdateTime(nullptr),
    mEndAtLoopSearchSetup(nullptr),
    mEndAtLoopSearch(nullptr),
    mEndAtLoopDetection(nullptr),
    mEndAtLoopClosure(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the value sequence metrics */
    this->mProcessTime = pMetricManager->AddValueSequence<int>(
        "Backend.ProcessTime");
    this->mProcessStepTime = pMetricManager->AddValueSequence<int>(
        "Backend.ProcessStepTime");
    this->mLoopSearchSetupTime = pMetricManager->AddValueSequence<int>(
        "Backend.LoopSearchSetupTime");
    this->mLoopSearchTime = pMetricManager->AddValueSequence<int>(
        "Backend.LoopSearchTime");
    this->mLoopDetectionSetupTime = pMetricManager->AddValueSequence<int>(
        "Backend.LoopDetectionSetupTime");
    this->mLoopDetectionTime = pMetricManager->AddValueSequence<int>(
        "Backend.LoopDetectionTime");
    this->mPoseGraphAppendTime = pMetricManager->AddValueSequence<int>(
        "Backend.PoseGraphAppendTime");
    this->mOptimizationSetupTime = pMetricManager->AddValueSequence<int>(
        "Backend.OptimizationSetupTime");
    this->mOptimizationTime = pMetricManager->AddValueSequence<int>(
        "Backend.OptimizationTime");
    this->mPoseGraphUpdateTime = pMetricManager->AddValueSequence<int>(
        "Backend.PoseGraphUpdateTime");

    this->mEndAtLoopSearchSetup = pMetricManager->AddValueSequence<int>(
        "Backend.EndAtLoopSearchSetup");
    this->mEndAtLoopSearch = pMetricManager->AddValueSequence<int>(
        "Backend.EndAtLoopSearch");
    this->mEndAtLoopDetection = pMetricManager->AddValueSequence<int>(
        "Backend.EndAtLoopDetection");
    this->mEndAtLoopClosure = pMetricManager->AddValueSequence<int>(
        "Backend.EndAtLoopClosure");
}

/* Constructor */
LidarGraphSlamBackend::LidarGraphSlamBackend(
    std::unique_ptr<PoseGraphOptimizer>&& poseGraphOptimizer,
    std::unique_ptr<LoopSearcher>&& loopSearcher,
    std::unique_ptr<LoopDetector>&& loopDetector) :
    mProcessCount(0),
    mPoseGraphOptimizer(std::move(poseGraphOptimizer)),
    mLoopSearcher(std::move(loopSearcher)),
    mLoopDetector(std::move(loopDetector)),
    mMetrics()
{
}

/* Run loop detection and pose graph optimization */
void LidarGraphSlamBackend::Run(
    LidarGraphSlam* const pParent,
    std::atomic<bool>& stopRequest)
{
    while (!stopRequest.load()) {
        /* Wait for the update from the SLAM frontend */
        pParent->WaitForNotification();

        /* Run a single iteration */
        this->RunStep(pParent);
    }

    /* Perform the last iteration after the SLAM frontend has finished
     * Loop detection and pose graph optimization are performed against
     * the last local grid map appended to the grid map builder */
    this->RunStep(pParent);
}

/* Run a single iteration */
void LidarGraphSlamBackend::RunStep(LidarGraphSlam* const pParent)
{
    /* Create the timer */
    Metric::Timer outerTimer;
    Metric::Timer timer;

    /* Update the number of the loop detection processes */
    this->mProcessCount += 1;

    /* Find loop candidates */
    const auto searchHint = pParent->GetLoopSearchHint();

    /* Update the metrics and restart the timer */
    this->mMetrics.mLoopSearchSetupTime->Observe(timer.ElapsedMicro());
    timer.Start();

    if (searchHint.mLocalMapNodes.empty() || searchHint.mScanNodes.empty()) {
        /* Update the metrics */
        this->mMetrics.mProcessTime->Observe(outerTimer.ElapsedMicro());
        this->mMetrics.mEndAtLoopSearchSetup->Observe(this->mProcessCount);
        return;
    }

    /* Search the loop candidates */
    auto loopCandidates = this->mLoopSearcher->Search(searchHint);

    /* Update the metrics and restart the timer */
    this->mMetrics.mLoopSearchTime->Observe(timer.ElapsedMicro());
    timer.Start();

    if (loopCandidates.empty()) {
        /* Update the metrics */
        this->mMetrics.mProcessTime->Observe(outerTimer.ElapsedMicro());
        this->mMetrics.mEndAtLoopSearch->Observe(this->mProcessCount);
        return;
    }

    /* Perform loop detection using candidates, each of which consists of
     * a collection of pose graph nodes and a local grid map */
    const LoopDetectionQueryVector loopDetectionQueries =
        pParent->GetLoopDetectionQueries(loopCandidates);

    /* Update the metrics and restart the timer */
    this->mMetrics.mLoopDetectionSetupTime->Observe(timer.ElapsedMicro());
    timer.Start();

    const LoopDetectionResultVector loopDetectionResults =
        this->mLoopDetector->Detect(loopDetectionQueries);

    /* Update the metrics and restart the timer */
    this->mMetrics.mLoopDetectionTime->Observe(timer.ElapsedMicro());
    timer.Start();

    if (loopDetectionResults.empty()) {
        /* Update the metrics */
        this->mMetrics.mProcessTime->Observe(outerTimer.ElapsedMicro());
        this->mMetrics.mEndAtLoopDetection->Observe(this->mProcessCount);
        return;
    }

    /* Append loop closing constraints using the loop detection results */
    pParent->AppendLoopClosingEdges(loopDetectionResults);

    /* Update the metrics and restart the timer */
    this->mMetrics.mPoseGraphAppendTime->Observe(timer.ElapsedMicro());
    timer.Start();

    /* Retrieve the finished pose graph nodes and edges */
    std::vector<LocalMapId> localMapIds;
    std::vector<Eigen::Vector3d> localMapPoses;
    std::vector<NodeId> scanNodeIds;
    std::vector<Eigen::Vector3d> scanPoses;
    std::vector<EdgePose> edgePoses;
    pParent->GetPoseGraphForOptimization(
        localMapIds, localMapPoses, scanNodeIds, scanPoses, edgePoses);

    /* Update the metrics */
    this->mMetrics.mOptimizationSetupTime->Observe(timer.ElapsedMicro());
    /* Notify that the SLAM backend optimization has started */
    pParent->NotifyOptimizationStarted();
    /* Restart the timer */
    timer.Start();

    /* Perform pose graph optimization */
    this->mPoseGraphOptimizer->Optimize(
        localMapPoses, scanPoses, edgePoses);

    /* Update the metrics and restart the timer */
    this->mMetrics.mOptimizationTime->Observe(timer.ElapsedMicro());
    timer.Start();

    /* Update pose graph nodes and rebuild grid maps */
    pParent->AfterLoopClosure(
        localMapIds, localMapPoses, scanNodeIds, scanPoses);

    /* Stop the timer */
    timer.Stop();
    /* Notify that the SLAM backend optimization has done */
    pParent->NotifyOptimizationDone();

    /* Update the metrics */
    this->mMetrics.mPoseGraphUpdateTime->Observe(timer.ElapsedMicro());
    this->mMetrics.mProcessTime->Observe(outerTimer.ElapsedMicro());
    this->mMetrics.mProcessStepTime->Observe(outerTimer.ElapsedMicro());
    this->mMetrics.mEndAtLoopClosure->Observe(this->mProcessCount);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
