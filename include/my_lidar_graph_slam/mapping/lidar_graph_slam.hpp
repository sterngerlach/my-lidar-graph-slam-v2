
/* lidar_graph_slam.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"
#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct LidarGraphSlamMetrics
{
    /* Constructor */
    LidarGraphSlamMetrics();
    /* Destructor */
    ~LidarGraphSlamMetrics() = default;

    /* Number of the loop constraints added to the pose graph */
    Metric::ValueSequenceBase* mNumOfNewLoopEdges;
};

/* Type definitions for convenience */
class LidarGraphSlam;
using LidarGraphSlamPtr = std::shared_ptr<LidarGraphSlam>;
using LidarGraphSlamWeakPtr = std::weak_ptr<LidarGraphSlam>;

/* Forward declarations */
class LidarGraphSlamBackend;
class LidarGraphSlamFrontend;

class LidarGraphSlam
{
public:
    /* Type definitions */
    using FrontendType = LidarGraphSlamFrontend;
    using BackendType = LidarGraphSlamBackend;

    /* Constructor */
    LidarGraphSlam(
        const std::shared_ptr<FrontendType>& slamFrontend,
        const std::shared_ptr<BackendType>& slamBackend,
        const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
        const std::shared_ptr<PoseGraph>& poseGraph);

    /* Destructor */
    ~LidarGraphSlam() = default;

    /* Copy constructor (disabled) */
    LidarGraphSlam(const LidarGraphSlam&) = delete;
    /* Copy assignment operator (disabled) */
    LidarGraphSlam& operator=(const LidarGraphSlam&) = delete;
    /* Move constructor (disabled) */
    LidarGraphSlam(LidarGraphSlam&&) = delete;
    /* Move assignment operator (disabled) */
    LidarGraphSlam& operator=(LidarGraphSlam&&) = delete;

    /* Process scan data and odometry */
    bool ProcessScan(const Sensor::ScanDataPtr<double>& rawScanData,
                     const RobotPose2D<double>& odomPose);

    /* Retrieve the total number of the processed input data */
    int ProcessCount() const;

    /* Retrieve the accumulated travel distance */
    double AccumTravelDist() const;

    /* Retrieve the full pose graph information */
    void GetPoseGraph(
        IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        IdMap<NodeId, ScanNode>& scanNodes,
        std::vector<PoseGraphEdge>& poseGraphEdges) const;
    /* Retrieve the finished pose graph for optimization */
    void GetPoseGraphForOptimization(
        std::vector<LocalMapId>& localMapIds,
        std::vector<Eigen::Vector3d>& localMapPoses,
        std::vector<NodeId>& scanNodeIds,
        std::vector<Eigen::Vector3d>& scanPoses,
        std::vector<EdgePose>& edgePoses) const;
    /* Retrieve the pose graph information */
    void GetPoseGraph(
        IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        IdMap<NodeId, ScanNodeData>& scanNodes,
        std::vector<EdgeData>& poseGraphEdges) const;

    /* Retrieve the latest data consisting of the pose from the last scan nodes
     * (last robot pose in a global coordinate frame), the latest map obtained
     * from the several recently acquired scans, the pose of the latest map
     * in a global coordinate frame, and the center position of the latest
     * map in a map-local coordinate frame */
    void GetLatestData(
        RobotPose2D<double>& lastScanPose,
        GridMap& latestMap,
        RobotPose2D<double>& latestMapPose,
        Point2D<double>& latestMapCenterPos) const;

    /* Retrieve the necessary information for loop search */
    LoopSearchHint GetLoopSearchHint() const;
    /* Retrieve the necessary information for loop detection */
    LoopDetectionQueryVector GetLoopDetectionQueries(
        const LoopCandidateVector& loopCandidates) const;

    /* Append a first node with an associated scan data and update the
     * current local grid map and the latest map */
    bool AppendFirstNodeAndEdge(
        const RobotPose2D<double>& initialScanPose,
        const Sensor::ScanDataPtr<double>& scanData);

    /* Append a new pose graph node and an odometry edge, and update the
     * current local grid map and the latest map */
    bool AppendNodeAndEdge(
        const RobotPose2D<double>& relativeScanPose,
        const Eigen::Matrix3d& edgeCovarianceMatrix,
        const Sensor::ScanDataPtr<double>& scanData);

    /* Append new loop closing edges */
    void AppendLoopClosingEdges(
        const LoopDetectionResultVector& loopDetectionResults);

    /* Rebuild grid maps after loop closure */
    void AfterLoopClosure(
        const std::vector<LocalMapId>& localMapIds,
        const std::vector<Eigen::Vector3d>& localMapPoses,
        const std::vector<NodeId>& scanNodeIds,
        const std::vector<Eigen::Vector3d>& scanPoses);

    /* Retrieve a latest map that contains latest scans */
    void GetLatestMap(RobotPose2D<double>& globalPose,
                      GridMap& latestMap,
                      NodeId& scanNodeIdMin,
                      NodeId& scanNodeIdMax) const;
    /* Build a global map that contains all local grid maps acquired */
    void GetGlobalMap(RobotPose2D<double>& globalPose,
                      GridMap& globalMap,
                      const NodeId scanNodeIdMin,
                      const NodeId scanNodeIdMax) const;
    /* Retrieve a collection of local grid maps */
    void GetLocalMaps(IdMap<LocalMapId, LocalMap>& localMaps);

    /* Start the SLAM backend */
    void StartBackend();
    /* Stop the SLAM backend */
    void StopBackend();
    /* Notify the SLAM backend */
    void NotifyBackend();
    /* Wait for the notification from the SLAM frontend */
    void WaitForNotification();

    /* Start the SLAM backend optimization */
    void NotifyOptimizationStarted();
    /* Notify that the SLAM backend optimization is done */
    void NotifyOptimizationDone();
    /* Wait for the SLAM backend optimization to be done */
    void WaitForOptimization();

private:
    /* SLAM frontend (scan matching and pose graph construction) */
    std::shared_ptr<FrontendType>   mFrontend;
    /* SLAM backend (loop detection and pose graph optimization) */
    std::shared_ptr<BackendType>    mBackend;
    /* Worker thread that runs SLAM backend */
    std::shared_ptr<std::thread>    mBackendThread;
    /* Flag to stop the SLAM backend */
    std::atomic<bool>               mBackendStopRequest;
    /* Condition variable for notification to SLAM backend */
    std::condition_variable         mBackendNotifyCond;
    /* Flag for notification to the SLAM backend */
    bool                            mBackendNotify;
    /* Condition variable to notify that SLAM backend optimization is done */
    std::condition_variable         mOptimizationDoneCond;
    /* Flag to indicate that the SLAM backend optimization is running */
    bool                            mOptimizationRunning;
    /* Grid map */
    std::shared_ptr<GridMapBuilder> mGridMapBuilder;
    /* Pose graph */
    std::shared_ptr<PoseGraph>      mPoseGraph;
    /* Shared mutex */
    mutable std::mutex              mMutex;
    /* Metrics information */
    LidarGraphSlamMetrics           mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP */
