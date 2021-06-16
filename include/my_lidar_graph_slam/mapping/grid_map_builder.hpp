
/* grid_map_builder.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP

#include <functional>
#include <map>
#include <vector>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_types.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct GridMapBuilderMetrics
{
    /* Constructor */
    GridMapBuilderMetrics();
    /* Destructor */
    ~GridMapBuilderMetrics() = default;

    /* Total processing time for updating the pose graph */
    Metric::ValueSequenceBase<int>*           mPoseGraphUpdateTime;
    /* Total processing time for updating the local grid map */
    Metric::ValueSequenceBase<int>*           mLocalMapUpdateTime;
    /* Total processing time for updating the latest grid map */
    Metric::ValueSequenceBase<int>*           mLatestMapUpdateTime;
    /* Travel distance since the last local map is created */
    Metric::ValueSequenceBase<float>*         mLocalMapIntervalTravelDist;
    /* Number of the local map nodes in the pose graph */
    Metric::ValueSequenceBase<int>*           mNumOfLocalMapNodes;
    /* Number of the edges in the pose graph */
    Metric::ValueSequenceBase<int>*           mNumOfEdges;
    /* Total memory consumption for the local grid maps */
    Metric::ValueSequenceBase<std::uint64_t>* mLocalMapMemoryUsage;
    /* Total memory consumption for the latest grid map */
    Metric::ValueSequenceBase<std::uint64_t>* mLatestMapMemoryUsage;
    /* Total memory consumption for the pose graph */
    Metric::ValueSequenceBase<std::uint64_t>* mPoseGraphMemoryUsage;
};

/*
 * LocalMapData struct stores information about the local map bounding box,
 * Ids of the scan nodes that reside in this local map, which are necessary
 * for the loop detection candidate search
 */
struct LocalMapData final
{
    /* Constructor */
    LocalMapData(const LocalMapId localMapId,
                 const Point2D<double>& globalMinPos,
                 const Point2D<double>& globalMaxPos,
                 const NodeId scanNodeIdMin,
                 const NodeId scanNodeIdMax,
                 const bool isFinished) :
        mId(localMapId),
        mGlobalMinPos(globalMinPos),
        mGlobalMaxPos(globalMaxPos),
        mScanNodeIdMin(scanNodeIdMin),
        mScanNodeIdMax(scanNodeIdMax),
        mFinished(isFinished) { }

    /* Destructor */
    ~LocalMapData() = default;

    /* Local map Id */
    const LocalMapId      mId;
    /* Minimum position of the local map (in a world frame) */
    const Point2D<double> mGlobalMinPos;
    /* Maximum position of the local map (in a world frame) */
    const Point2D<double> mGlobalMaxPos;
    /* Minimum scan node Id inside this local map */
    const NodeId          mScanNodeIdMin;
    /* Maximum scan node Id inside this local map */
    const NodeId          mScanNodeIdMax;
    /* Flag to represent whether this local map is finished */
    const bool            mFinished;
};

/*
 * LocalMap struct keeps necessary information for a local grid map
 * A single local grid map consists of the sequence of scan data
 * within the range from `mScanNodeIdMin` to `mScanNodeIdMax` and has
 * an associated local map Id `mId`, from which the local map pose in a
 * world coordinate is obtained using the pose graph
 */
struct LocalMap final
{
    /* Constructor */
    LocalMap(const LocalMapId localMapId,
             GridMap&& gridMap,
             const NodeId scanNodeId) :
        mId(localMapId),
        mMap(std::move(gridMap)),
        mScanNodeIdMin(scanNodeId),
        mScanNodeIdMax(scanNodeId),
        mFinished(false) { }

    /* Destructor */
    ~LocalMap() = default;

    /* Copy constructor */
    LocalMap(const LocalMap&) = default;
    /* Copy assignment operator */
    LocalMap& operator=(const LocalMap&) = default;
    /* Move constructor */
    LocalMap(LocalMap&&) noexcept = default;
    /* Move assignment operator */
    LocalMap& operator=(LocalMap&&) noexcept = default;

    /* Inspect the memory usage in bytes */
    inline std::uint64_t InspectMemoryUsage() const {
        return this->mMap.InspectMemoryUsage() +
               sizeof(this->mId) +
               sizeof(this->mScanNodeIdMin) +
               sizeof(this->mScanNodeIdMax) +
               sizeof(this->mFinished); }

    /* Local map Id */
    const LocalMapId mId;
    /* Local grid map consisting of the sequence of the scan data
     * from `mScanNodeIdMin` to `mScanNodeIdMax` */
    GridMap          mMap;
    /* Minimum scan node Id */
    NodeId           mScanNodeIdMin;
    /* Maximum scan node Id */
    NodeId           mScanNodeIdMax;
    /* Flags to represent whether this local map is finished and will not
     * be changed (no more scan data is added) */
    bool             mFinished;
};

/*
 * GridMapBuilder class is responsible for creating and updating grid maps
 */
class GridMapBuilder
{
public:
    /* Constructor */
    GridMapBuilder(const double mapResolution,
                   const int patchSize,
                   const int numOfScansForLatestMap,
                   const double travelDistThreshold,
                   const std::size_t numOfOverlappedScans,
                   const double usableRangeMin,
                   const double usableRangeMax,
                   const double probHit,
                   const double probMiss);

    /* Destructor */
    ~GridMapBuilder() = default;

    /* Append the new scan data */
    bool AppendScan(std::shared_ptr<PoseGraph>& poseGraph,
                    const RobotPose2D<double>& relativeScanPose,
                    const Eigen::Matrix3d& scanPoseCovarianceMatrix,
                    const Sensor::ScanDataPtr<double>& scanData);

    /* Re-create the local grid maps and latest map after the loop closure */
    void AfterLoopClosure(const std::shared_ptr<PoseGraph>& poseGraph);

    /* Finish the current local grid map and compute the center position
     * of the local grid map in the map-local coordinate frame */
    void FinishLocalMap(const std::shared_ptr<PoseGraph>& poseGraph);

    /* Construct the global map */
    void ConstructGlobalMap(const std::shared_ptr<PoseGraph>& poseGraph,
                            const NodeId scanNodeIdMin,
                            const NodeId scanNodeIdMax,
                            RobotPose2D<double>& globalMapPose,
                            GridMap& globalMap);

    /* Append a new local map */
    void AppendLocalMap(std::shared_ptr<PoseGraph>& poseGraph,
                        const RobotPose2D<double>& scanPose,
                        const Eigen::Matrix3d& scanPoseCovarianceMatrix,
                        const NodeId scanNodeId);

    /* Update the pose graph, add a new scan node and create a new local grid
     * map (and its corresponding new local map node) if necessary */
    bool UpdatePoseGraph(std::shared_ptr<PoseGraph>& poseGraph,
                         const RobotPose2D<double>& relativeScanPose,
                         const Eigen::Matrix3d& scanPoseCovarianceMatrix,
                         const Sensor::ScanDataPtr<double>& scanData);

    /* Update the grid map (list of the local grid maps) */
    void UpdateGridMap(const std::shared_ptr<PoseGraph>& poseGraph);

    /* Update the grid map with the latest scans */
    void UpdateLatestMap(const IdMap<NodeId, ScanNode>& scanNodes);

    /* Update the accumulated travel distance after the loop closure */
    void UpdateAccumTravelDist(const IdMap<NodeId, ScanNode>& scanNodes);

    /* Retrieve the local grid maps */
    inline const IdMap<LocalMapId, LocalMap>& LocalMaps() const
    { return this->mLocalMaps; }

    /* Retrieve the local map information of the specified index */
    inline LocalMap& LocalMapAt(LocalMapId localMapId)
    { return this->mLocalMaps.at(localMapId); }
    /* Retrieve the local map information of the specified index */
    inline const LocalMap& LocalMapAt(LocalMapId localMapId) const
    { return this->mLocalMaps.at(localMapId); }

    /* Retrieve the latest local map information */
    LocalMap& LatestLocalMap();
    /* Retrieve the latest local map information */
    const LocalMap& LatestLocalMap() const;

    /* Retrieve the grid map constructed from the latest scans */
    inline const GridMap& LatestMap() const
    { return this->mLatestMap; }
    /* Retrieve the latest map pose in a world frame */
    inline const RobotPose2D<double>& LatestMapPose() const
    { return this->mLatestMapPose; }

    /* Get the accumulated travel distance */
    inline double AccumTravelDist() const { return this->mAccumTravelDist; }

    /* Get the minimum Id of the latest scan nodes */
    inline NodeId LatestScanIdMin() const { return this->mLatestScanIdMin; }
    /* Get the maximum Id of the latest scan nodes */
    inline NodeId LatestScanIdMax() const { return this->mLatestScanIdMax; }

private:
    /* Construct the grid map from the specified scans */
    void ConstructMapFromScans(
        const RobotPose2D<double>& globalMapPose,
        GridMap& gridMap,
        const IdMap<NodeId, ScanNode>& scanNodes,
        const NodeId scanNodeIdMin,
        const NodeId scanNodeIdMax) const;

    /* Construct the grid map from all the scans */
    void ConstructMapFromAllScans(
        const RobotPose2D<double>& globalMapPose,
        GridMap& gridMap,
        const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        const IdMap<NodeId, ScanNode>& scanNodes);

    /* Compute the bounding box of the scan and scan points in a local frame */
    void ComputeBoundingBoxAndScanPointsMapLocal(
        const RobotPose2D<double>& globalMapPose,
        const RobotPose2D<double>& globalScanPose,
        const Sensor::ScanDataPtr<double>& scanData,
        Point2D<double>& localMinPos,
        Point2D<double>& localMaxPos,
        std::vector<Point2D<double>>& localHitPoints);

    /* Compute the indices of the missed grid cells
     * using Bresenham algorithm */
    void ComputeMissedGridCellIndices(
        const Point2D<int>& startGridCellIdx,
        const Point2D<int>& endGridCellIdx,
        std::vector<Point2D<int>>& gridCellIndices) const;

    /* Compute the indices of the missed grid cells using the Bresenham
     * algorithm at the subpixel accuracy */
    void ComputeMissedIndicesScaled(
        const Point2D<int>& scaledStartIdx,
        const Point2D<int>& scaledEndIdx,
        const int subpixelScale,
        std::vector<Point2D<int>>& missedIndices) const;

private:
    /* Subpixel scale for computing the missed grid cell indices */
    static constexpr int SubpixelScale = 100;

private:
    /* Map resolution (in meters) */
    const double                mResolution;
    /* Patch size (in the number of grid cells) */
    const int                   mPatchSize;
    /* Vector of the local grid maps */
    IdMap<LocalMapId, LocalMap> mLocalMaps;
    /* Grid map constructed from the latest scans
     * Used for scan matching and updated when a new scan data is available */
    GridMap                     mLatestMap;
    /* Latest map pose in a world frame */
    RobotPose2D<double>         mLatestMapPose;
    /* Accumulated travel distance */
    double                      mAccumTravelDist;
    /* The number of scans used to construct the latest map */
    const int                   mNumOfScansForLatestMap;
    /* The minimum Id of the latest scan nodes */
    NodeId                      mLatestScanIdMin;
    /* The maximum Id of the latest scan nodes */
    NodeId                      mLatestScanIdMax;
    /* Last robot pose in a world frame */
    RobotPose2D<double>         mLastRobotPose;
    /* Accumulated travel distance since the last local grid map is created */
    double                      mTravelDistLastLocalMap;
    /* Robot pose in a world frame when the last local grid map is created */
    RobotPose2D<double>         mRobotPoseLastLocalMap;
    /* Travel distance threshold for creating a new local grid map */
    const double                mTravelDistThreshold;
    /* Number of the overlapped scans between consecutive local maps */
    const std::size_t           mNumOfOverlappedScans;
    /* Minimum range of the laser scan that is considered valid */
    const double                mUsableRangeMin;
    /* Maximum range of the laser scan that is considered valid */
    const double                mUsableRangeMax;
    /* Occupancy probability value for hit grid cell
     * Used for calculating the probability value with Binary Bayes Filter */
    const double                mProbHit;
    /* Occupancy probability value for missed grid cell
     * Used for calculating the probability value with Binary Bayes Filter */
    const double                mProbMiss;
    /* Metrics information */
    GridMapBuilderMetrics       mMetrics;
};

/*
 * Utility function declarations
 */

/* Compute the maximum of a 'winSize' pixel wide row at each pixel */
void SlidingWindowMaxRow(const GridMap& gridMap,
                         ConstMap& intermediateMap,
                         const int winSize);

/* Compute the maximum of a 'winSize' pixel wide column at each pixel */
void SlidingWindowMaxCol(const ConstMap& intermediateMap,
                         ConstMap& precompMap,
                         const int winSize);

/* Precompute coarser grid maps for efficiency */
void PrecomputeGridMaps(const GridMap& gridMap,
                        std::vector<ConstMap>& precompMaps,
                        const int nodeHeightMax);

/* Precompute grid map for efficiency */
ConstMap PrecomputeGridMap(const GridMap& gridMap,
                           ConstMap& intermediateMap,
                           const int winSize);

/* Precompute grid map for efficiency */
ConstMap PrecomputeGridMap(const GridMap& gridMap,
                           const int winSize);

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP */
