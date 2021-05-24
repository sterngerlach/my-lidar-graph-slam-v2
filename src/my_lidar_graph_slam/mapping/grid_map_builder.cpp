
/* grid_map_builder.cpp */

#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * GridMapBuilderMetrics struct implementations
 */

/* Constructor */
GridMapBuilderMetrics::GridMapBuilderMetrics() :
    mPoseGraphUpdateTime(nullptr),
    mLocalMapUpdateTime(nullptr),
    mLatestMapUpdateTime(nullptr),
    mLocalMapIntervalTravelDist(nullptr),
    mNumOfLocalMapNodes(nullptr),
    mNumOfEdges(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the distribution metrics */
    this->mPoseGraphUpdateTime = pMetricManager->AddDistribution(
        "GridMapBuilder.PoseGraphUpdateTime");
    this->mLocalMapUpdateTime = pMetricManager->AddDistribution(
        "GridMapBuilder.LocalMapUpdateTime");
    this->mLatestMapUpdateTime = pMetricManager->AddDistribution(
        "GridMapBuilder.LatestMapUpdateTime");
    this->mLocalMapIntervalTravelDist = pMetricManager->AddDistribution(
        "GridMapBuilder.LocalMapIntervalTravelDist");

    /* Register the value sequence metrics */
    this->mNumOfLocalMapNodes = pMetricManager->AddValueSequenceInt(
        "GridMapBuilder.NumOfLocalMapNodes");
    this->mNumOfEdges = pMetricManager->AddValueSequenceInt(
        "GridMapBuilder.NumOfEdges");
}

/*
 * GridMapBuilder class implementations
 */

/* Constructor */
GridMapBuilder::GridMapBuilder(
    const double mapResolution,
    const int patchSize,
    const int numOfScansForLatestMap,
    const double travelDistThreshold,
    const std::size_t numOfOverlappedScans,
    const double usableRangeMin,
    const double usableRangeMax,
    const double probHit,
    const double probMiss) :
    mResolution(mapResolution),
    mPatchSize(patchSize),
    mLatestMap(mapResolution, patchSize, 0, 0, Point2D<double>(0.0, 0.0)),
    mLatestMapPose(0.0, 0.0, 0.0),
    mAccumTravelDist(0.0),
    mNumOfScansForLatestMap(numOfScansForLatestMap),
    mLatestScanIdMin(0),
    mLatestScanIdMax(0),
    mLastRobotPose(0.0, 0.0, 0.0),
    mTravelDistLastLocalMap(0.0),
    mRobotPoseLastLocalMap(0.0, 0.0, 0.0),
    mTravelDistThreshold(travelDistThreshold),
    mNumOfOverlappedScans(numOfOverlappedScans),
    mUsableRangeMin(usableRangeMin),
    mUsableRangeMax(usableRangeMax),
    mProbHit(probHit),
    mProbMiss(probMiss),
    mMetrics()
{
}

/* Retrieve the latest local map information */
LocalMap& GridMapBuilder::LatestLocalMap()
{
    /* Use the const version of the method */
    const auto* pThis = static_cast<const GridMapBuilder*>(this);
    return const_cast<LocalMap&>(pThis->LatestLocalMap());
}

/* Retrieve the latest local map information */
const LocalMap& GridMapBuilder::LatestLocalMap() const
{
    /* Make sure that the local maps are not empty */
    Assert(!this->mLocalMaps.empty());
    /* Return the local map with the largest Id since it is the latest one */
    return this->mLocalMaps.Back();
}

/* Append the new scan data */
bool GridMapBuilder::AppendScan(
    IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    IdMap<NodeId, ScanNode>& scanNodes,
    std::vector<PoseGraphEdge>& poseGraphEdges,
    const RobotPose2D<double>& relativeScanPose,
    const Eigen::Matrix3d& scanPoseCovarianceMatrix,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Update the pose graph and create a new local grid map if necessary */
    const bool localMapInserted = this->UpdatePoseGraph(
        localMapNodes, scanNodes, poseGraphEdges,
        relativeScanPose, scanPoseCovarianceMatrix, scanData);
    /* Update the grid map */
    this->UpdateGridMap(localMapNodes, scanNodes);
    /* Return whether the new local map is created */
    return localMapInserted;
}

/* Re-create the local grid maps and latest map after the loop closure */
void GridMapBuilder::AfterLoopClosure(
    const IdMap<LocalMapId, LocalMapNode>& /* localMapNodes */,
    const IdMap<NodeId, ScanNode>& scanNodes)
{
    /* Update the accumulated travel distance */
    this->UpdateAccumTravelDist(scanNodes);
}

/* Finish the current local grid map and compute the center position
 * of the local grid map in the map-local coordinate frame */
void GridMapBuilder::FinishLocalMap(
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>& scanNodes)
{
    if (this->mLocalMaps.empty())
        return;

    /* Retrieve the latest local map */
    auto& localMap = this->LatestLocalMap();
    /* Retrieve the latest local map node */
    const auto& localMapNode = localMapNodes.Back();
    /* Make sure that their Ids are the same */
    Assert(localMap.mId == localMapNode.mLocalMapId);

    /* The current local grid map is marked as finished */
    localMap.mFinished = true;
}

/* Construct the global grid map */
void GridMapBuilder::ConstructGlobalMap(
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>& scanNodes,
    const NodeId scanNodeIdMin,
    const NodeId scanNodeIdMax,
    RobotPose2D<double>& globalMapPose,
    GridMapType& globalMap)
{
    /* World coordinate pose of the first scan node is used as the map pose */
    const RobotPose2D<double> mapPose = scanNodes.Front().mGlobalPose;
    /* The above pose is the center of the local map coordinate */
    const Point2D<double> localCenterPos = Point2D<double>::Zero;

    /* Construct the global map */
    GridMapType gridMap { this->mResolution, this->mPatchSize,
                          0, 0, localCenterPos };
    this->ConstructMapFromScans(mapPose, gridMap, scanNodes,
                                scanNodeIdMin, scanNodeIdMax);

    /* Set the result */
    globalMapPose = mapPose;
    globalMap = std::move(gridMap);

    return;
}

/* Append a new local map */
void GridMapBuilder::AppendLocalMap(
    IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    IdMap<NodeId, ScanNode>& scanNodes,
    std::vector<PoseGraphEdge>& poseGraphEdges,
    const RobotPose2D<double>& scanPose,
    const Eigen::Matrix3d& scanPoseCovarianceMatrix,
    const NodeId scanNodeId)
{
    /* The latest local map is marked as finished and the center position
     * of the local map in the map-local coordinate frame is computed */
    this->FinishLocalMap(localMapNodes, scanNodes);

    /* Determine the Id of the new local map */
    const LocalMapId localMapId = localMapNodes.empty() ?
        LocalMapId { 0 } :
        LocalMapId { localMapNodes.Back().mLocalMapId.mId + 1 };
    /* Pose of the new local map is as same as the scan pose */
    const RobotPose2D<double>& localMapPose = scanPose;

    /* Create the odometry edge connecting the old local map and the new
     * scan node if necessary */
    if (!this->mLocalMaps.empty()) {
        /* Retrieve the old local map */
        const auto& oldLocalMap = this->LatestLocalMap();
        const auto& oldLocalMapNode = localMapNodes.Back();
        /* Make sure that their Ids are the same */
        Assert(oldLocalMap.mId == oldLocalMapNode.mLocalMapId);
        /* Make sure that the old local map is finished */
        Assert(oldLocalMap.mFinished);
        /* Make sure that the old local map contains scan data older than
         * the new scan data with the Id `scanNodeId` */
        Assert(scanNodeId > oldLocalMap.mScanNodeIdMax);

        /* Append the inter-map odometry edge for the given scan node */
        const RobotPose2D<double> mapLocalScanPose =
            NormalizeAngle(InverseCompound(
                oldLocalMapNode.mGlobalPose, scanPose));

        /* Compute the covariance matrix */
        const Eigen::Matrix3d mapLocalCovarianceMat =
            ConvertCovarianceFromWorldToLocal(
                oldLocalMapNode.mGlobalPose, scanPoseCovarianceMatrix);
        /* Compute an information matrix */
        const Eigen::Matrix3d mapLocalInformationMat =
            mapLocalCovarianceMat.inverse();

        /* Append the odometry edge connecting the old local map and
         * the new scan node */
        poseGraphEdges.emplace_back(
            oldLocalMapNode.mLocalMapId, scanNodeId,
            EdgeType::InterLocalMap, ConstraintType::Odometry,
            mapLocalScanPose, mapLocalInformationMat);
    }

    /* Insert a new local map node to the pose graph */
    /* We use the pose (in a world coordinate frame) from the newly
     * inserted scan node as the pose for a newly inserted local map */
    localMapNodes.Append(localMapId, localMapPose);

    /* Current robot pose in a world frame is used as the
     * origin of the local map coordinate */
    const Point2D<double> centerPos { 0.0, 0.0 };
    /* Create a new local map */
    GridMapType newLocalMap {
        this->mResolution, this->mPatchSize, 0, 0, centerPos };

    if (!this->mLocalMaps.empty()) {
        /* Retrieve the reference to the last local map */
        const auto& lastLocalMap = this->LatestLocalMap();

        /* Determine the number of scans used for initializing a local map */
        const std::size_t numOfScanNodes = std::min(
            scanNodes.size(), this->mNumOfOverlappedScans);

        /* Retrieve the last scan Id used for the last local map */
        const NodeId lastMapNodeIdMax = lastLocalMap.mScanNodeIdMax;
        /* Determine the scan data used for initializing a local map */
        const auto lastNodeIt = scanNodes.IteratorAt(lastMapNodeIdMax);
        const auto firstNodeIt = std::prev(lastNodeIt, numOfScanNodes);
        const NodeId nodeIdMin = firstNodeIt->mId;
        const NodeId nodeIdMax = lastNodeIt->mId;

        /* Initialize a local map with multiple recent scans */
        this->ConstructMapFromScans(localMapPose, newLocalMap, scanNodes,
                                    nodeIdMin, nodeIdMax);
    }

    /* Insert a new local map */
    this->mLocalMaps.Append(localMapId, std::move(newLocalMap), scanNodeId);

    /* Update the metrics */
    this->mMetrics.mLocalMapIntervalTravelDist->Observe(
        this->mTravelDistLastLocalMap);

    /* Reset the variables properly */
    this->mTravelDistLastLocalMap = 0.0;
    this->mRobotPoseLastLocalMap = localMapPose;

    return;
}

/* Update the pose graph, add a new scan node and create a new local grid
 * map (and its corresponding new local map node) if necessary */
bool GridMapBuilder::UpdatePoseGraph(
    IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    IdMap<NodeId, ScanNode>& scanNodes,
    std::vector<PoseGraphEdge>& poseGraphEdges,
    const RobotPose2D<double>& relativeScanPose,
    const Eigen::Matrix3d& scanPoseCovarianceMatrix,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Create the new timer */
    Metric::Timer timer;

    /* Id of the new scan node to be added */
    const NodeId scanNodeId = scanNodes.empty() ?
        NodeId { 0 } : NodeId { scanNodes.Back().mNodeId.mId + 1 };
    /* Latest scan node pose in a world coordinate frame */
    const RobotPose2D<double> prevScanPose = scanNodes.empty() ?
        RobotPose2D<double>(0.0, 0.0, 0.0) :
        scanNodes.Back().mGlobalPose;
    /* Compute a new scan node pose using the latest node pose
     * `prevScanNodePose` here, since the pose of the latest node
     * (starting node of the new odometry edge) might have been modified
     * by the loop closure, which is performed in the SLAM backend */
    const RobotPose2D<double> scanPose =
        Compound(prevScanPose, relativeScanPose);

    /* Update the accumulated travel distance */
    this->mAccumTravelDist += Distance(relativeScanPose);
    /* Update the accumulated travel distance since the last grid map */
    this->mTravelDistLastLocalMap += Distance(relativeScanPose);

    /* Determine whether to create a new local map */
    const bool travelDistThreshold =
        this->mTravelDistLastLocalMap >= this->mTravelDistThreshold;
    const bool lastMapFinished =
        !this->mLocalMaps.empty() && this->mLocalMaps.Back().mFinished;
    const bool isFirstScan = this->mLocalMaps.size() == 0;
    const bool localMapInserted =
        travelDistThreshold || lastMapFinished || isFirstScan;

    /* Create a new local map if necessary
     * The pose (in a world coordinate frame) of the new local map is set to
     * the robot pose `scanPose` when the scan data `scanData` is acquired */
    if (localMapInserted)
        this->AppendLocalMap(localMapNodes, scanNodes, poseGraphEdges,
                             scanPose, scanPoseCovarianceMatrix, scanNodeId);

    /* Make sure that the local maps are not empty for now */
    Assert(!this->mLocalMaps.empty());
    Assert(!localMapNodes.empty());

    /* Retrieve the Id of the latest local map */
    const auto& latestLocalMap = this->LatestLocalMap();
    const auto& latestLocalMapNode = localMapNodes.Back();
    /* Make sure that their Ids are the same */
    Assert(latestLocalMap.mId == latestLocalMapNode.mLocalMapId);
    /* Make sure that we can insert the new scan to the latest local map */
    Assert(!latestLocalMap.mFinished);
    Assert(scanNodeId >= latestLocalMap.mScanNodeIdMin);

    /* Compute the map-local pose of the new scan */
    /* Angular component is normalized from -pi to pi */
    const RobotPose2D<double> mapLocalScanPose =
        NormalizeAngle(InverseCompound(
            latestLocalMapNode.mGlobalPose, scanPose));

    /* Append the new scan node */
    scanNodes.Append(scanNodeId, latestLocalMap.mId, mapLocalScanPose,
                     scanData, scanPose);

    /* Covariance matrix must be rotated beforehand since the matrix
     * must represent the covariance in the map-local coordinate frame
     * (not world coordinate frame) */
    const Eigen::Matrix3d mapLocalCovarianceMat =
        ConvertCovarianceFromWorldToLocal(
            latestLocalMapNode.mGlobalPose, scanPoseCovarianceMatrix);
    /* Calculate an information matrix by inverting a covariance matrix
     * obtained from the scan matching */
    const Eigen::Matrix3d mapLocalInformationMat =
        mapLocalCovarianceMat.inverse();

    /* Append the new pose graph edge connecting the latest grid map and the
     * latest scan node */
    poseGraphEdges.emplace_back(
        latestLocalMapNode.mLocalMapId, scanNodeId,
        EdgeType::IntraLocalMap, ConstraintType::Odometry,
        mapLocalScanPose, mapLocalInformationMat);

    /* Update the metrics */
    this->mMetrics.mPoseGraphUpdateTime->Observe(timer.ElapsedMicro());
    this->mMetrics.mNumOfLocalMapNodes->Observe(localMapNodes.size());
    this->mMetrics.mNumOfEdges->Observe(poseGraphEdges.size());

    return localMapInserted;
}

/* Update the grid map (list of the local grid maps) */
void GridMapBuilder::UpdateGridMap(
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>& scanNodes)
{
    /* Vector for storing missed grid cell indices
     * Specified as static variable to reduce the performance loss
     * caused by memory allocations and deallocations */
    static std::vector<Point2D<int>> missedGridCellIndices;

    /* Create the timer */
    Metric::Timer timer;

    /* Retrieve the latest local map to which the latest scan is added */
    auto& latestLocalMap = this->LatestLocalMap();
    const auto& latestLocalMapNode = localMapNodes.Back();
    /* Retrieve the latest scan node */
    const auto& latestScanNode = scanNodes.Back();

    /* Make sure that their Ids are the same */
    Assert(latestLocalMap.mId == latestLocalMapNode.mLocalMapId);
    /* Make sure that the latest scan belongs to the latest local map */
    Assert(latestLocalMap.mId == latestScanNode.mLocalMapId);
    /* Make sure that we can insert the new scan into the latest local map */
    Assert(!latestLocalMap.mFinished);
    Assert(latestScanNode.mNodeId >= latestLocalMap.mScanNodeIdMin);

    /* Local grid map */
    auto& localMap = latestLocalMap.mMap;
    /* Local map pose in a world coordinate frame */
    const RobotPose2D<double>& globalMapPose = latestLocalMapNode.mGlobalPose;
    /* Scan pose in a world coordinate frame */
    const RobotPose2D<double>& globalScanPose = latestScanNode.mGlobalPose;
    /* Latest scan data */
    const Sensor::ScanDataPtr<double>& scanData = latestScanNode.mScanData;

    /* Compute the scan points and bounding box */
    Point2D<double> localMinPos;
    Point2D<double> localMaxPos;
    std::vector<Point2D<double>> localHitPoints;
    this->ComputeBoundingBoxAndScanPointsMapLocal(
        globalMapPose, globalScanPose, scanData,
        localMinPos, localMaxPos, localHitPoints);

    /* Expand the local map so that it can contain the latest scan */
    localMap.Expand(localMinPos.mX, localMinPos.mY,
                    localMaxPos.mX, localMaxPos.mY);

    /* Compute the sensor pose from the robot pose */
    const RobotPose2D<double> globalSensorPose =
        Compound(globalScanPose, scanData->RelativeSensorPose());
    /* Compute the sensor pose in a map-local coordinate frame */
    /* We can compute this using `latestScanNode.mLocalPose` and
     * `scanData->RelativeSensorPose()` since the relative poses between
     * the latest local map node and the scan nodes inside this local map are
     * not changed; the latest local map is not finished and thus relative
     * poses are not updated by the loop closure in SLAM backend */
    const RobotPose2D<double> localSensorPose =
        InverseCompound(globalMapPose, globalSensorPose);
    /* Calculate the grid cell index corresponding to the sensor pose */
    const Point2D<int> sensorGridCellIdx =
        localMap.LocalPosToGridCellIndex(
            localSensorPose.mX, localSensorPose.mY);

    /* Integrate the scan into the local map */
    const std::size_t numOfFilteredScans = localHitPoints.size();

    for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
        /* Compute the index of the hit grid cell */
        const Point2D<int> hitGridCellIdx =
            localMap.LocalPosToGridCellIndex(
                localHitPoints[i].mX, localHitPoints[i].mY);

        /* Compute the indices of the missed grid cells */
        this->ComputeMissedGridCellIndices(
            sensorGridCellIdx, hitGridCellIdx, missedGridCellIndices);

        /* Update occupancy probability values for missed grid cells */
        for (std::size_t j = 0; j < missedGridCellIndices.size(); ++j)
            localMap.Update(missedGridCellIndices[j], this->mProbMiss);

        /* Update occupancy probability values for hit grid cell */
        localMap.Update(hitGridCellIdx, this->mProbHit);
    }

    /* Update the scan Id information of the local map */
    latestLocalMap.mScanNodeIdMax = latestScanNode.mNodeId;

    /* Update the metrics */
    this->mMetrics.mLocalMapUpdateTime->Observe(timer.ElapsedMicro());

    return;
}

/* Update the grid map with the latest scans */
void GridMapBuilder::UpdateLatestMap(
    const IdMap<NodeId, ScanNode>& scanNodes)
{
    /* Create the timer */
    Metric::Timer timer;

    /* Make sure that the pose graph is not empty */
    Assert(!scanNodes.empty());

    /* Get the iterator pointing to the scan nodes used for latest maps */
    const int numOfScansForMap = std::min(
        static_cast<int>(scanNodes.size()), this->mNumOfScansForLatestMap);
    const auto lastNodeIt = std::prev(scanNodes.end());
    const auto latestNodeIt = std::prev(lastNodeIt, numOfScansForMap - 1);

    /* Validate the scan node Ids */
    Assert(latestNodeIt->mId <= lastNodeIt->mId);

    /* Update the minimum and maximum scan node Id */
    this->mLatestScanIdMin = latestNodeIt->mId;
    this->mLatestScanIdMax = lastNodeIt->mId;
    /* Update the pose of the latest map */
    this->mLatestMapPose = latestNodeIt->mData.mGlobalPose;

    /* Update the latest map */
    this->ConstructMapFromScans(
        this->mLatestMapPose, this->mLatestMap, scanNodes,
        this->mLatestScanIdMin, this->mLatestScanIdMax);

    /* Update the metrics */
    this->mMetrics.mLatestMapUpdateTime->Observe(timer.ElapsedMicro());

    return;
}

/* Update the accumulated travel distance after the loop closure */
void GridMapBuilder::UpdateAccumTravelDist(
    const IdMap<NodeId, ScanNode>& scanNodes)
{
    /* Reset the accumulated travel distance */
    this->mAccumTravelDist = 0.0;

    /* Retrieve the two iterators pointing to the first scan node and
     * the node following to the last scan node */
    auto nodeIt = scanNodes.begin();
    auto endIt = scanNodes.end();
    auto nextIt = std::next(nodeIt);

    /* Return if the scan nodes are empty or only one scan node exists
     * `nextIt` is invalid and undefined behaviour if there is no node */
    if (nodeIt == endIt || nextIt == endIt)
        return;

    /* Accumulate the travel distance using pose graph nodes */
    for (; nextIt != endIt; ++nodeIt, ++nextIt) {
        const RobotPose2D<double>& nodePose = nodeIt->mData.mGlobalPose;
        const RobotPose2D<double>& nextNodePose = nextIt->mData.mGlobalPose;
        this->mAccumTravelDist += Distance(nodePose, nextNodePose);
    }
}

/* Construct the grid map from the specified scans */
void GridMapBuilder::ConstructMapFromScans(
    const RobotPose2D<double>& globalMapPose,
    GridMapType& gridMap,
    const IdMap<NodeId, ScanNode>& scanNodes,
    const NodeId scanNodeIdMin,
    const NodeId scanNodeIdMax) const
{
    /* Vector for storing missed grid cell indices
     * Specified as static variable to reduce the performance loss
     * caused by memory allocations and deallocations */
    static std::vector<Point2D<int>> missedGridCellIndices;

    /* Get the iterators to the scan nodes */
    auto firstNodeIt = scanNodes.lower_bound(scanNodeIdMin);
    auto lastNodeIt = scanNodes.upper_bound(scanNodeIdMax);

    /* Make sure that at least one scan data is used to build a map
     * Otherwise the bounding box below is not updated and causes a overflow */
    Assert(firstNodeIt != lastNodeIt);

    /* Compute the scan points and bounding box in a grid map frame */
    Point2D<double> localMinPos { std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max() };
    Point2D<double> localMaxPos { std::numeric_limits<double>::min(),
                                  std::numeric_limits<double>::min() };
    std::map<NodeId, std::vector<Point2D<double>>> localHitPoints;

    for (auto nodeIt = firstNodeIt; nodeIt != lastNodeIt; ++nodeIt) {
        /* Retrieve the pose and scan data in the scan node */
        const NodeId scanNodeId = nodeIt->mId;
        const auto& scanNode = nodeIt->mData;
        const RobotPose2D<double>& globalNodePose = scanNode.mGlobalPose;
        const auto& scanData = scanNode.mScanData;

        /* Compute the global sensor pose from the node pose */
        const RobotPose2D<double> globalSensorPose =
            Compound(globalNodePose, scanData->RelativeSensorPose());
        /* Compute the local sensor pose */
        const RobotPose2D<double> localSensorPose =
            InverseCompound(globalMapPose, globalSensorPose);

        localMinPos.mX = std::min(localMinPos.mX, localSensorPose.mX);
        localMinPos.mY = std::min(localMinPos.mY, localSensorPose.mY);
        localMaxPos.mX = std::max(localMaxPos.mX, localSensorPose.mX);
        localMaxPos.mY = std::max(localMaxPos.mY, localSensorPose.mY);

        /* Compute the minimum and maximum range of the scan */
        const double minRange = std::max(
            this->mUsableRangeMin, scanData->MinRange());
        const double maxRange = std::min(
            this->mUsableRangeMax, scanData->MaxRange());

        /* Compute the scan points and bounding box */
        const std::size_t numOfScans = scanData->NumOfScans();
        std::vector<Point2D<double>> nodeLocalHitPoints;
        nodeLocalHitPoints.reserve(numOfScans);

        for (std::size_t i = 0; i < numOfScans; ++i) {
            const double scanRange = scanData->RangeAt(i);

            if (scanRange >= maxRange || scanRange <= minRange)
                continue;

            /* Compute the hit point in a local frame */
            const Point2D<double> localHitPoint =
                scanData->HitPoint(localSensorPose, i);
            nodeLocalHitPoints.push_back(localHitPoint);

            /* Update the bounding box */
            localMinPos.mX = std::min(localMinPos.mX, localHitPoint.mX);
            localMinPos.mY = std::min(localMinPos.mY, localHitPoint.mY);
            localMaxPos.mX = std::max(localMaxPos.mX, localHitPoint.mX);
            localMaxPos.mY = std::max(localMaxPos.mY, localHitPoint.mY);
        }

        localHitPoints.emplace(scanNodeId, std::move(nodeLocalHitPoints));
    }

    /* Expand the area to prevent floating-point errors */
    localMinPos.mX -= gridMap.Resolution();
    localMinPos.mY -= gridMap.Resolution();
    localMaxPos.mX += gridMap.Resolution();
    localMaxPos.mY += gridMap.Resolution();

    /* Create a new grid map that contains all scan points */
    gridMap.Resize(localMinPos.mX, localMinPos.mY,
                   localMaxPos.mX, localMaxPos.mY);
    gridMap.Reset();

    /* Integrate the scan into the grid map
     * Reuse the same iterators as above since the pose graph is constant */
    for (auto nodeIt = firstNodeIt; nodeIt != lastNodeIt; ++nodeIt) {
        /* Retrieve the pose and scan data in the scan node */
        const NodeId scanNodeId = nodeIt->mId;
        const auto& scanNode = nodeIt->mData;
        const RobotPose2D<double>& globalNodePose = scanNode.mGlobalPose;
        const auto& scanData = scanNode.mScanData;

        /* Compute the global sensor pose from the node pose */
        const RobotPose2D<double> globalSensorPose =
            Compound(globalNodePose, scanData->RelativeSensorPose());
        /* Compute the map local sensor pose */
        const RobotPose2D<double> localSensorPose =
            InverseCompound(globalMapPose, globalSensorPose);
        /* Compute the grid cell index corresponding to the sensor pose */
        const Point2D<int> sensorGridCellIdx =
            gridMap.LocalPosToGridCellIndex(
                localSensorPose.mX, localSensorPose.mY);

        /* Integrate the scan into the grid map */
        const std::size_t numOfFilteredScans =
            localHitPoints[scanNodeId].size();

        for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
            /* Retrieve the hit point in a local frame */
            const Point2D<double>& localHitPoint =
                localHitPoints[scanNodeId].at(i);
            /* Compute the index of the hit grid cell */
            const Point2D<int> hitGridCellIdx =
                gridMap.LocalPosToGridCellIndex(
                    localHitPoint.mX, localHitPoint.mY);

            /* Compute the indices of the missed grid cells */
            this->ComputeMissedGridCellIndices(
                sensorGridCellIdx, hitGridCellIdx, missedGridCellIndices);

            /* Update occupancy probability values for missed grid cells */
            for (std::size_t j = 0; j < missedGridCellIndices.size(); ++j)
                gridMap.Update(missedGridCellIndices[j], this->mProbMiss);

            /* Update occupancy probability values for hit grid cell */
            gridMap.Update(hitGridCellIdx, this->mProbHit);
        }
    }

    return;
}

/* Construct the grid map from all the scans */
void GridMapBuilder::ConstructMapFromAllScans(
    const RobotPose2D<double>& globalMapPose,
    GridMapType& gridMap,
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>& scanNodes)
{
    /* Vector for storing missed grid cell indices
     * Specified as static variable to reduce the performance loss
     * caused by memory allocations and deallocations */
    static std::vector<Point2D<int>> missedGridCellIndices;

    /* Make sure that the pose graph nodes are not empty */
    Assert(!localMapNodes.empty());
    Assert(!scanNodes.empty());

    /* Compute the scan points and bounding box
     * in the map-local coordinate frame */
    Point2D<double> localMinPos { std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max() };
    Point2D<double> localMaxPos { std::numeric_limits<double>::min(),
                                  std::numeric_limits<double>::min() };
    std::map<NodeId, RobotPose2D<double>> localSensorPoses;
    std::map<NodeId, std::vector<Point2D<double>>> localHitPoints;

    for (const auto& [localMapId, localMapNode] : localMapNodes) {
        /* Retrieve the local grid map */
        const auto& localMap = this->mLocalMaps.at(localMapId);
        /* Retrieve the iterators to the scan nodes in the local grid map */
        const auto firstScanIt = scanNodes.IteratorAt(localMap.mScanNodeIdMin);
        const auto lastNodeIt = scanNodes.IteratorAt(localMap.mScanNodeIdMax);
        const auto scanRange = scanNodes.RangeFromIterator(
            firstScanIt, std::next(lastNodeIt));

        for (const auto& [scanNodeId, scanNode] : scanRange) {
            /* Compute the global sensor pose from the scan node pose */
            const RobotPose2D<double> globalSensorPose =
                Compound(localMapNode.mGlobalPose, scanNode.mLocalPose);
            /* Compute the sensor pose in the map-local coordinate frame */
            const RobotPose2D<double> localSensorPose =
                InverseCompound(globalMapPose, globalSensorPose);

            localMinPos.mX = std::min(localMinPos.mX, localSensorPose.mX);
            localMinPos.mY = std::min(localMinPos.mY, localSensorPose.mY);
            localMaxPos.mX = std::max(localMaxPos.mX, localSensorPose.mX);
            localMaxPos.mY = std::max(localMaxPos.mY, localSensorPose.mY);

            /* Compute the minimum and maximum range of the scan */
            const double minRange = std::max(
                this->mUsableRangeMin, scanNode.mScanData->MinRange());
            const double maxRange = std::min(
                this->mUsableRangeMax, scanNode.mScanData->MaxRange());

            /* Compute the scan points and bounding box */
            const std::size_t numOfScans = scanNode.mScanData->NumOfScans();
            std::vector<Point2D<double>> nodeLocalHitPoints;
            nodeLocalHitPoints.reserve(numOfScans);

            for (std::size_t i = 0; i < numOfScans; ++i) {
                const double scanRange = scanNode.mScanData->RangeAt(i);

                if (scanRange >= maxRange || scanRange <= minRange)
                    continue;

                /* Compute the hit point in a local frame */
                const Point2D<double> localHitPoint =
                    scanNode.mScanData->HitPoint(localSensorPose, i);
                nodeLocalHitPoints.push_back(localHitPoint);

                /* Update the bounding box */
                localMinPos.mX = std::min(localMinPos.mX, localHitPoint.mX);
                localMinPos.mY = std::min(localMinPos.mY, localHitPoint.mY);
                localMaxPos.mX = std::max(localMaxPos.mX, localHitPoint.mX);
                localMaxPos.mY = std::max(localMaxPos.mY, localHitPoint.mY);
            }

            localSensorPoses.emplace(scanNodeId, localSensorPose);
            localHitPoints.emplace(scanNodeId, std::move(nodeLocalHitPoints));
        }
    }

    /* Expand the grid map bounding box to prevent out-of-bound memory accesses
     * caused by floating-point errors */
    localMinPos.mX -= gridMap.Resolution();
    localMinPos.mY -= gridMap.Resolution();
    localMaxPos.mX += gridMap.Resolution();
    localMaxPos.mY += gridMap.Resolution();

    /* Create a new grid map that contains all scan points */
    gridMap.Resize(localMinPos.mX, localMinPos.mY,
                   localMaxPos.mX, localMaxPos.mY);
    gridMap.Reset();

    /* Integrate the scan into the grid map */
    for (const auto& [scanNodeId, localSensorPose] : localSensorPoses) {
        /* Compute the grid cell index corresponding to the sensor pose */
        const Point2D<int> sensorGridCellIdx =
            gridMap.LocalPosToGridCellIndex(
                localSensorPose.mX, localSensorPose.mY);
        /* Retrieve the scan points in the map-local coordinate frame */
        const auto& nodeLocalHitPoints = localHitPoints.at(scanNodeId);

        for (std::size_t i = 0; i < nodeLocalHitPoints.size(); ++i) {
            /* Retrieve the hit point in a map-local coordinate frame */
            const Point2D<double>& localHitPoint = nodeLocalHitPoints.at(i);
            /* Compute the index of the hit grid cell */
            const Point2D<int> hitGridCellIdx =
                gridMap.LocalPosToGridCellIndex(
                    localHitPoint.mX, localHitPoint.mY);
            /* Compute the indices of the missed grid cells */
            this->ComputeMissedGridCellIndices(
                sensorGridCellIdx, hitGridCellIdx, missedGridCellIndices);

            /* Update occupancy probability values for missed grid cells */
            for (std::size_t j = 0; j < missedGridCellIndices.size(); ++j)
                gridMap.Update(missedGridCellIndices[j], this->mProbMiss);

            /* Update occupancy probability values for the hit grid cell */
            gridMap.Update(hitGridCellIdx, this->mProbHit);
        }
    }

    return;
}

/* Compute the bounding box of the scan and scan points in a local frame */
void GridMapBuilder::ComputeBoundingBoxAndScanPointsMapLocal(
    const RobotPose2D<double>& globalMapPose,
    const RobotPose2D<double>& globalScanPose,
    const Sensor::ScanDataPtr<double>& scanData,
    Point2D<double>& localMinPos,
    Point2D<double>& localMaxPos,
    std::vector<Point2D<double>>& localHitPoints)
{
    /* Compute the sensor pose from the robot pose */
    const RobotPose2D<double> globalSensorPose =
        Compound(globalScanPose, scanData->RelativeSensorPose());
    /* Compute the map local sensor pose */
    const RobotPose2D<double> localSensorPose =
        InverseCompound(globalMapPose, globalSensorPose);

    /* Initialize the minimum coordinate of the given scan */
    localMinPos.mX = localSensorPose.mX;
    localMinPos.mY = localSensorPose.mY;

    /* Initialize the maximum coordinate of the given scan */
    localMaxPos.mX = localSensorPose.mX;
    localMaxPos.mY = localSensorPose.mY;

    const std::size_t numOfScans = scanData->NumOfScans();
    localHitPoints.reserve(numOfScans);

    /* Minimum and maximum range of the scan */
    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());

    /* Calculate the bounding box and scan points in a local frame */
    for (std::size_t i = 0; i < numOfScans; ++i) {
        const double scanRange = scanData->RangeAt(i);

        if (scanRange >= maxRange || scanRange <= minRange)
            continue;

        /* Calculate the hit point in a local frame */
        const Point2D<double> localHitPoint =
            scanData->HitPoint(localSensorPose, i);
        localHitPoints.push_back(localHitPoint);

        /* Update the corner positions (bounding box) */
        localMinPos.mX = std::min(localMinPos.mX, localHitPoint.mX);
        localMinPos.mY = std::min(localMinPos.mY, localHitPoint.mY);
        localMaxPos.mX = std::max(localMaxPos.mX, localHitPoint.mX);
        localMaxPos.mY = std::max(localMaxPos.mY, localHitPoint.mY);
    }

    return;
}

/* Compute the indices of the missed grid cells
 * using Bresenham algorithm */
void GridMapBuilder::ComputeMissedGridCellIndices(
    const Point2D<int>& startGridCellIdx,
    const Point2D<int>& endGridCellIdx,
    std::vector<Point2D<int>>& gridCellIndices) const
{
    /* Clear the grid cell indices */
    gridCellIndices.clear();
    /* Use Bresenham algorithm for computing indices */
    Bresenham(startGridCellIdx, endGridCellIdx, gridCellIndices);
    /* Remove the last item since it is the hit grid cell */
    gridCellIndices.pop_back();
}

/*
 * Utility function implementations
 */

/* Compute the maximum of a 'winSize' pixel wide row at each pixel */
void SlidingWindowMaxRow(const GridMapType& gridMap,
                         ConstMapType& intermediateMap,
                         const int winSize)
{
    /* Each grid cell in the grid map stores an occupancy probability value
     * using the data type `StorageType` (defined as std::uint16_t) */
    using StorageType = GridMapType::StorageType;

    /* Compute the maximum for each column */
    const StorageType unknownRawValue = gridMap.UnknownRawValue();
    int colIdx = 0;

    std::function<StorageType(int)> inFunc =
        [&colIdx, &gridMap, unknownRawValue](int rowIdx) {
        return gridMap.RawValue(colIdx, rowIdx, unknownRawValue); };

    std::function<void(int, StorageType)> outFunc =
        [&colIdx, &gridMap, &intermediateMap, unknownRawValue](
            int rowIdx, StorageType maxValue) {
        const Point2D<int> patchIdx =
            gridMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated = gridMap.PatchIsAllocated(patchIdx);

        if (isAllocated)
            intermediateMap.SetRawValue(colIdx, rowIdx, maxValue); };

    const int numOfGridCellsX = gridMap.NumOfGridCellsX();
    const int numOfGridCellsY = gridMap.NumOfGridCellsY();

    /* Apply the sliding window maximum function */
    for (colIdx = 0; colIdx < numOfGridCellsX; ++colIdx)
        SlidingWindowMax(inFunc, outFunc, numOfGridCellsY, winSize);
}

/* Compute the maximum of a 'winSize' pixel wide column at each pixel */
void SlidingWindowMaxCol(const ConstMapType& intermediateMap,
                         ConstMapType& precompMap,
                         const int winSize)
{
    /* Each grid cell in the grid map stores an occupancy probability value
     * using the data type `StorageType` (defined as std::uint16_t) */
    using StorageType = GridMapType::StorageType;

    /* Compute the maximum for each row */
    const StorageType unknownRawValue = intermediateMap.UnknownRawValue();
    int rowIdx = 0;

    std::function<StorageType(int)> inFunc =
        [&rowIdx, &intermediateMap, unknownRawValue](int colIdx) {
        return intermediateMap.RawValue(colIdx, rowIdx, unknownRawValue); };

    std::function<void(int, StorageType)> outFunc =
        [&rowIdx, &intermediateMap, &precompMap, unknownRawValue](
            int colIdx, StorageType maxValue) {
        const Point2D<int> patchIdx =
            intermediateMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated = intermediateMap.PatchIsAllocated(patchIdx);

        if (isAllocated)
            precompMap.SetRawValue(colIdx, rowIdx, maxValue); };

    const int numOfGridCellsX = intermediateMap.NumOfGridCellsX();
    const int numOfGridCellsY = intermediateMap.NumOfGridCellsY();

    /* Apply the sliding window maximum function */
    for (rowIdx = 0; rowIdx < numOfGridCellsY; ++rowIdx)
        SlidingWindowMax(inFunc, outFunc, numOfGridCellsX, winSize);
}

/* Precompute coarser grid maps for efficiency */
void PrecomputeGridMaps(const GridMapType& gridMap,
                        std::vector<ConstMapType>& precomputedMaps,
                        const int nodeHeightMax)
{
    /* Create the temporary grid map to store the intermediate result
     * The map size is as same as the local grid map and is reused for
     * several times below */
    ConstMapType intermediateMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Clear the precomputed coarser grid maps */
    precomputedMaps.clear();

    /* Compute a grid map for each node height */
    for (int nodeHeight = 0, winSize = 1;
         nodeHeight <= nodeHeightMax; ++nodeHeight, winSize <<= 1) {
        /* Precompute a grid map */
        ConstMapType precompMap =
            PrecomputeGridMap(gridMap, intermediateMap, winSize);

        /* Append the newly created map */
        precomputedMaps.emplace_back(std::move(precompMap));
    }
}

/* Precompute grid map for efficiency */
ConstMapType PrecomputeGridMap(const GridMapType& gridMap,
                               ConstMapType& intermediateMap,
                               const int winSize)
{
    /* Create a new grid map
     * Each pixel stores the maximum of the occupancy probability values of
     * 'winSize' * 'winSize' box of pixels beginning there */
    ConstMapType precompMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Store the maximum of the 'winSize' pixel wide row */
    SlidingWindowMaxRow(gridMap, intermediateMap, winSize);
    /* Store the maximum of the 'winSize' pixel wide column */
    SlidingWindowMaxCol(intermediateMap, precompMap, winSize);

    return precompMap;
}

/* Precompute grid map for efficiency */
ConstMapType PrecomputeGridMap(const GridMapType& gridMap,
                               const int winSize)
{
    /* Create a temporary map to store the intermediate result */
    ConstMapType intermediateMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Create a new grid map */
    ConstMapType precompMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Store the maximum of the 'winSize' pixel wide row */
    SlidingWindowMaxRow(gridMap, intermediateMap, winSize);
    /* Store the maximum of the 'winSize' pixel wide column */
    SlidingWindowMaxCol(intermediateMap, precompMap, winSize);

    return precompMap;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
