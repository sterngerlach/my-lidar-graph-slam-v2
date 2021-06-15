
/* lidar_graph_slam_backend.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_BACKEND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_BACKEND_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct BackendMetrics
{
    /* Constructor */
    BackendMetrics();
    /* Destructor */
    ~BackendMetrics() = default;

    /* Total processing time of the SLAM backend */
    Metric::CounterBase*            mProcessTime;
    /* Total processing time of the one step of the SLAM backend */
    Metric::ValueSequenceBase<int>* mProcessStepTime;
    /* Total processing time for setting up the loop search */
    Metric::ValueSequenceBase<int>* mLoopSearchSetupTime;
    /* Total processing time for the loop search */
    Metric::ValueSequenceBase<int>* mLoopSearchTime;
    /* Total processing time for setting up the loop detection */
    Metric::ValueSequenceBase<int>* mLoopDetectionSetupTime;
    /* Total processing time for the loop detection */
    Metric::ValueSequenceBase<int>* mLoopDetectionTime;
    /* Total processing time for adding loop constraints to the pose graph */
    Metric::ValueSequenceBase<int>* mPoseGraphAppendTime;
    /* Total processing time for setting up the pose graph optimization */
    Metric::ValueSequenceBase<int>* mOptimizationSetupTime;
    /* Total processing time for the pose graph optimization */
    Metric::ValueSequenceBase<int>* mOptimizationTime;
    /* Total processing time for updating the pose graph */
    Metric::ValueSequenceBase<int>* mPoseGraphUpdateTime;

    /* Collections of iteration counters, in that iteration,
     * information for the loop detection was empty */
    Metric::ValueSequenceBase<int>* mEndAtLoopSearchSetup;
    /* Collections of iteration counters, in that iteration,
     * loop candidates were not found */
    Metric::ValueSequenceBase<int>* mEndAtLoopSearch;
    /* Collections of iteration counters, in that iteration,
     * loop was not detected */
    Metric::ValueSequenceBase<int>* mEndAtLoopDetection;
    /* Collections of iteration counters, in that iteration,
     * loop detection and graph optimization were performed */
    Metric::ValueSequenceBase<int>* mEndAtLoopClosure;
};

class LidarGraphSlamBackend
{
public:
    /* Constructor */
    LidarGraphSlamBackend(
        std::unique_ptr<PoseGraphOptimizer>&& poseGraphOptimizer,
        std::unique_ptr<LoopSearcher>&& loopSearcher,
        std::unique_ptr<LoopDetector>&& loopDetector);
    /* Destructor */
    ~LidarGraphSlamBackend() = default;

    /* Copy constructor (disabled) */
    LidarGraphSlamBackend(const LidarGraphSlamBackend&) = delete;
    /* Copy assignment operator (disabled) */
    LidarGraphSlamBackend& operator=(const LidarGraphSlamBackend&) = delete;
    /* Move constructor */
    LidarGraphSlamBackend(LidarGraphSlamBackend&&) = default;
    /* Move assignment operator */
    LidarGraphSlamBackend& operator=(LidarGraphSlamBackend&&) = default;

    /* Run loop detection and pose graph optimization */
    void Run(LidarGraphSlam* const pParent,
             std::atomic<bool>& stopRequest);

    /* Run a single iteration */
    void RunStep(LidarGraphSlam* const pParent);

private:
    /* Total number of the loop detection processes */
    int                                 mProcessCount;
    /* Pose graph optimizer */
    std::unique_ptr<PoseGraphOptimizer> mPoseGraphOptimizer;
    /* Loop searcher */
    std::unique_ptr<LoopSearcher>       mLoopSearcher;
    /* Loop detector */
    std::unique_ptr<LoopDetector>       mLoopDetector;
    /* Metrics information */
    BackendMetrics                      mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_BACKEND_HPP */
