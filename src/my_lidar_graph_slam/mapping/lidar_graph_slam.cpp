
/* lidar_graph_slam.cpp */

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/LU>

#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_frontend.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlamMetrics::LidarGraphSlamMetrics() :
    mNumOfNewLoopEdges(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the value sequence metrics */
    this->mNumOfNewLoopEdges = pMetricManager->AddValueSequence<int>(
        "LidarGraphSlam.NumOfNewLoopEdges");
}

/* Constructor */
LidarGraphSlam::LidarGraphSlam(
    const std::shared_ptr<FrontendType>& slamFrontend,
    const std::shared_ptr<BackendType>& slamBackend,
    const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
    const std::shared_ptr<PoseGraph>& poseGraph) :
    mFrontend(slamFrontend),
    mBackend(slamBackend),
    mBackendThread(nullptr),
    mBackendStopRequest(false),
    mBackendNotifyCond(),
    mBackendNotify(false),
    mOptimizationDoneCond(),
    mOptimizationRunning(false),
    mGridMapBuilder(gridMapBuilder),
    mPoseGraph(poseGraph),
    mMetrics()
{
}

/* Process scan data and odometry */
bool LidarGraphSlam::ProcessScan(
    const Sensor::ScanDataPtr<double>& rawScanData,
    const RobotPose2D<double>& odomPose)
{
    /* Process the latest scan data and odometry information in frontend */
    return this->mFrontend->ProcessScan(this, rawScanData, odomPose);
}

/* Retrieve the total number of the processed input data */
int LidarGraphSlam::ProcessCount() const
{
    return this->mFrontend->ProcessCount();
}

/* Retrieve the accumulated travel distance */
double LidarGraphSlam::AccumTravelDist() const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Return the accumulated travel distance from the grid map builder */
    return this->mGridMapBuilder->AccumTravelDist();
}

/* Retrieve the full pose graph information */
void LidarGraphSlam::GetPoseGraph(
    IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    IdMap<NodeId, ScanNode>& scanNodes,
    std::vector<PoseGraphEdge>& poseGraphEdges) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    localMapNodes.clear();
    scanNodes.clear();
    poseGraphEdges.clear();

    /* Copy the local map nodes */
    for (const auto& [nodeId, mapNode] : this->mPoseGraph->LocalMapNodes())
        localMapNodes.Append(nodeId, mapNode.mGlobalPose);

    /* Copy the scan nodes */
    for (const auto& [nodeId, scanNode] : this->mPoseGraph->ScanNodes())
        scanNodes.Append(nodeId, scanNode.mLocalMapId, scanNode.mLocalPose,
                         scanNode.mScanData, scanNode.mGlobalPose);

    /* Copy the pose graph edges */
    for (const auto& poseGraphEdge : this->mPoseGraph->Edges())
        poseGraphEdges.emplace_back(
            poseGraphEdge.mLocalMapNodeId, poseGraphEdge.mScanNodeId,
            poseGraphEdge.mEdgeType, poseGraphEdge.mConstraintType,
            poseGraphEdge.mRelativePose, poseGraphEdge.mInformationMat);
}

/* Retrieve the finished pose graph for optimization */
void LidarGraphSlam::GetPoseGraphForOptimization(
    std::vector<LocalMapId>& localMapIds,
    std::vector<Eigen::Vector3d>& localMapPoses,
    std::vector<NodeId>& scanNodeIds,
    std::vector<Eigen::Vector3d>& scanPoses,
    std::vector<EdgePose>& edgePoses) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    const auto& localMapNodes = this->mPoseGraph->LocalMapNodes();
    const auto& scanNodes = this->mPoseGraph->ScanNodes();
    const auto& poseGraphEdges = this->mPoseGraph->Edges();

    localMapIds.reserve(localMapNodes.size());
    localMapPoses.reserve(localMapNodes.size());
    scanNodeIds.reserve(scanNodes.size());
    scanPoses.reserve(scanNodes.size());
    edgePoses.reserve(poseGraphEdges.size());

    /* Get the iterator to the first unfinished local map
     * If not found, all local grid maps stored in the grid map builder are
     * in finished state and return them */
    const auto unfinishedMapIt = std::find_if(
        this->mGridMapBuilder->LocalMaps().cbegin(),
        this->mGridMapBuilder->LocalMaps().cend(),
        [](const IdMap<LocalMapId, LocalMap>::ConstIdDataPair& localMapPair) {
            return !localMapPair.mData.mFinished; });

    /* Local maps with Ids larger than `localMapIdMax` are removed */
    const LocalMapId localMapIdMax =
        unfinishedMapIt != this->mGridMapBuilder->LocalMaps().cend() ?
            unfinishedMapIt->mId : LocalMapId { LocalMapId::Invalid };
    /* Scan nodes with Ids larger than `nodeIdMax` are removed */
    const NodeId nodeIdMax =
        unfinishedMapIt != this->mGridMapBuilder->LocalMaps().cend() ?
            unfinishedMapIt->mData.mScanNodeIdMin : NodeId { NodeId::Invalid };

    /* Copy the local map nodes */
    for (const auto& [nodeId, mapNode] : localMapNodes) {
        if (localMapIdMax.mId == LocalMapId::Invalid ||
            mapNode.mLocalMapId < localMapIdMax) {
            localMapIds.emplace_back(mapNode.mLocalMapId);
            localMapPoses.emplace_back(mapNode.mGlobalPose.mX,
                                       mapNode.mGlobalPose.mY,
                                       mapNode.mGlobalPose.mTheta);
        }
    }

    /* Copy the scan nodes */
    for (const auto& [nodeId, scanNode] : scanNodes) {
        if (localMapIdMax.mId == LocalMapId::Invalid ||
            (scanNode.mLocalMapId < localMapIdMax &&
             scanNode.mNodeId < nodeIdMax)) {
            scanNodeIds.emplace_back(scanNode.mNodeId);
            scanPoses.emplace_back(scanNode.mGlobalPose.mX,
                                   scanNode.mGlobalPose.mY,
                                   scanNode.mGlobalPose.mTheta);
        }
    }

    /* Copy the pose graph edges */
    for (const auto& poseGraphEdge : poseGraphEdges) {
        if (localMapIdMax.mId == LocalMapId::Invalid ||
            (poseGraphEdge.mLocalMapNodeId < localMapIdMax &&
             poseGraphEdge.mScanNodeId < nodeIdMax)) {
            const auto localMapNodeIt = localMapNodes.IteratorAt(
                poseGraphEdge.mLocalMapNodeId);
            const auto scanNodeIt = scanNodes.IteratorAt(
                poseGraphEdge.mScanNodeId);
            const int localMapNodeIdx = static_cast<int>(
                std::distance(localMapNodes.begin(), localMapNodeIt));
            const int scanNodeIdx = static_cast<int>(
                std::distance(scanNodes.begin(), scanNodeIt));
            const Eigen::Vector3d relativePose {
                poseGraphEdge.mRelativePose.mX,
                poseGraphEdge.mRelativePose.mY,
                poseGraphEdge.mRelativePose.mTheta };

            edgePoses.emplace_back(
                poseGraphEdge.IsLoopClosingConstraint(),
                localMapNodeIdx, scanNodeIdx,
                relativePose, poseGraphEdge.mInformationMat);
        }
    }
}

/* Retrieve the pose graph information */
void LidarGraphSlam::GetPoseGraph(
    IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    IdMap<NodeId, ScanNodeData>& scanNodes,
    std::vector<EdgeData>& poseGraphEdges) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    poseGraphEdges.reserve(this->mPoseGraph->Edges().size());

    /* Setup the local map nodes information */
    for (const auto& [localMapId, localMapNode] :
         this->mPoseGraph->LocalMapNodes())
        /* Append the local map node information */
        localMapNodes.Append(localMapId, localMapNode.mGlobalPose);

    /* Setup the scan nodes information */
    for (const auto& [scanNodeId, scanNode] : this->mPoseGraph->ScanNodes())
        /* Append the scan node information */
        scanNodes.Append(scanNodeId, scanNode.mGlobalPose);

    /* Setup the pose graph edges information */
    for (const auto& edge : this->mPoseGraph->Edges())
        poseGraphEdges.emplace_back(
            edge.mLocalMapNodeId, edge.mScanNodeId,
            edge.mEdgeType, edge.mConstraintType);
}

/* Retrieve the latest data */
void LidarGraphSlam::GetLatestData(
    RobotPose2D<double>& lastScanPose,
    GridMap& latestMap,
    RobotPose2D<double>& latestMapPose,
    Point2D<double>& latestMapCenterPos) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Update the grid map with latest scans */
    this->mGridMapBuilder->UpdateLatestMap(
        this->mPoseGraph->ScanNodes());

    /* Set the last pose from the pose graph in a world coordinate frame */
    lastScanPose = this->mPoseGraph->ScanNodes().Back().mGlobalPose;
    /* Set the latest grid map and its pose in a world coordinate frame */
    latestMap = this->mGridMapBuilder->LatestMap();
    latestMapPose = this->mGridMapBuilder->LatestMapPose();

    /* Set the center position of the latest map
     * in a map-local coordinate frame */
    /* Get the range of the scan nodes in the latest map */
    const auto firstNodeIt = this->mPoseGraph->ScanNodes().IteratorAt(
        this->mGridMapBuilder->LatestScanIdMin());
    const auto lastNodeIt = this->mPoseGraph->ScanNodes().IteratorAt(
        this->mGridMapBuilder->LatestScanIdMax());
    const auto endNodeIt = std::next(lastNodeIt);
    const auto nodeRange = this->mPoseGraph->ScanNodes().RangeFromIterator(
        firstNodeIt, endNodeIt);
    const auto numOfNodes = std::distance(firstNodeIt, endNodeIt);

    /* Accumulate the positions of the scan nodes in the latest map */
    latestMapCenterPos = Point2D<double>::Zero;

    for (const auto& [scanNodeId, scanNode] : nodeRange) {
        /* Compute the map-local pose of the scan node */
        const RobotPose2D<double> mapLocalNodePose =
            InverseCompound(this->mGridMapBuilder->LatestMapPose(),
                            scanNode.mGlobalPose);
        latestMapCenterPos.mX += mapLocalNodePose.mX;
        latestMapCenterPos.mY += mapLocalNodePose.mY;
    }

    /* Compute the center position of the latest map */
    latestMapCenterPos.mX /= static_cast<double>(numOfNodes);
    latestMapCenterPos.mY /= static_cast<double>(numOfNodes);
}

/* Retrieve the necessary information for loop search */
LoopSearchHint LidarGraphSlam::GetLoopSearchHint() const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Get the iterator to the first unfinished local map
     * If not found, all local grid maps stored in the grid map builder are
     * in finished state and return them */
    const auto unfinishedMapIt = std::find_if(
        this->mGridMapBuilder->LocalMaps().cbegin(),
        this->mGridMapBuilder->LocalMaps().cend(),
        [](const IdMap<LocalMapId, LocalMap>::ConstIdDataPair& localMapPair) {
            return !localMapPair.mData.mFinished; });

    /* Return if there is no finished local grid map */
    if (unfinishedMapIt == this->mGridMapBuilder->LocalMaps().begin())
        return LoopSearchHint {
            IdMap<NodeId, ScanNodeData>(),
            IdMap<LocalMapId, LocalMapData>(),
            this->mGridMapBuilder->AccumTravelDist(),
            NodeId(NodeId::Invalid), LocalMapId(LocalMapId::Invalid) };

    /* Local maps with Ids larger than `localMapIdMax` are ignored */
    const LocalMapId localMapIdMax =
        unfinishedMapIt != this->mGridMapBuilder->LocalMaps().cend() ?
            unfinishedMapIt->mId : LocalMapId { LocalMapId::Invalid };
    /* Scan nodes with Ids larger than `nodeIdMax` are ignored */
    const NodeId nodeIdMax =
        unfinishedMapIt != this->mGridMapBuilder->LocalMaps().cend() ?
            unfinishedMapIt->mData.mScanNodeIdMin : NodeId { NodeId::Invalid };

    IdMap<NodeId, ScanNodeData> scanNodes;
    IdMap<LocalMapId, LocalMapData> localMapNodes;

    /* Setup the scan nodes information */
    for (const auto& [nodeId, scanNode] : this->mPoseGraph->ScanNodes()) {
        /* Ignore the scan node that belongs to the unfinished local map */
        if (localMapIdMax.mId != LocalMapId::Invalid &&
            (scanNode.mLocalMapId >= localMapIdMax ||
             scanNode.mNodeId >= nodeIdMax))
            break;

        /* Append the scan node information */
        scanNodes.Append(nodeId, scanNode.mGlobalPose);
    }

    /* Setup the local map nodes information */
    for (const auto& [nodeId, mapNode] : this->mPoseGraph->LocalMapNodes()) {
        /* Ignore the unfinished local map */
        if (localMapIdMax.mId != LocalMapId::Invalid &&
            nodeId >= localMapIdMax)
            break;

        /* Retrieve the local map information */
        const auto& localMap = this->mGridMapBuilder->LocalMapAt(nodeId);
        const auto& gridMap = localMap.mMap;
        const auto& globalPose = mapNode.mGlobalPose;

        /* Compute the bounding box of the local map in a world frame */
        const int rows = gridMap.Rows();
        const int cols = gridMap.Cols();

        const std::array<Point2D<double>, 4> cornerPoints {
            Compound(globalPose, gridMap.IndexToPosition(0, 0)),
            Compound(globalPose, gridMap.IndexToPosition(rows, 0)),
            Compound(globalPose, gridMap.IndexToPosition(0, cols)),
            Compound(globalPose, gridMap.IndexToPosition(rows, cols)) };

        const Point2D<double> globalMinPos {
            std::min(std::min(cornerPoints[0].mX, cornerPoints[1].mX),
                     std::min(cornerPoints[2].mX, cornerPoints[3].mX)),
            std::min(std::min(cornerPoints[0].mY, cornerPoints[1].mY),
                     std::min(cornerPoints[2].mY, cornerPoints[3].mY)) };
        const Point2D<double> globalMaxPos {
            std::max(std::max(cornerPoints[0].mX, cornerPoints[1].mX),
                     std::max(cornerPoints[2].mX, cornerPoints[3].mX)),
            std::max(std::max(cornerPoints[0].mY, cornerPoints[1].mY),
                     std::max(cornerPoints[2].mY, cornerPoints[3].mY)) };

        /* Append the local map node information */
        localMapNodes.Append(nodeId, globalMinPos, globalMaxPos,
                             localMap.mScanNodeIdMin, localMap.mScanNodeIdMax,
                             localMap.mFinished);
    }

    /* Retrieve the last finished local map */
    const auto& lastFinishedMapNode =
        this->mPoseGraph->LocalMapNodes().at(localMapNodes.IdMax());
    const auto& lastFinishedMap =
        this->mGridMapBuilder->LocalMapAt(lastFinishedMapNode.mLocalMapId);
    /* Retrieve the scan node in the last finished local map */
    const NodeId scanNodeId {
        (lastFinishedMap.mScanNodeIdMin.mId +
         lastFinishedMap.mScanNodeIdMax.mId) / 2 };
    const auto& lastFinishedScanNode =
        this->mPoseGraph->ScanNodes().at(scanNodeId);

    /* Make sure that `lastFinishedScanNode` belongs to the last finished
     * local map `lastFinishedMap` */
    Assert(lastFinishedScanNode.mLocalMapId == lastFinishedMap.mId);
    Assert(lastFinishedScanNode.mNodeId >= lastFinishedMap.mScanNodeIdMin &&
           lastFinishedScanNode.mNodeId <= lastFinishedMap.mScanNodeIdMax);

    /* Return the necessary information for loop search */
    return LoopSearchHint {
        std::move(scanNodes), std::move(localMapNodes),
        this->mGridMapBuilder->AccumTravelDist(),
        lastFinishedScanNode.mNodeId, lastFinishedMapNode.mLocalMapId };
}

/* Retrieve the necessary information for loop detection */
LoopDetectionQueryVector LidarGraphSlam::GetLoopDetectionQueries(
    const LoopCandidateVector& loopCandidates) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    LoopDetectionQueryVector loopDetectionQueries;
    loopDetectionQueries.reserve(loopCandidates.size());

    /* Setup the data needed for loop detection */
    for (const auto& loopCandidate : loopCandidates) {
        /* Retrieve the reference to the query scan node */
        const auto& queryNode = this->mPoseGraph->ScanNodes().at(
            loopCandidate.mQueryScanNodeId);
        /* Retrieve the reference to the reference scan node */
        const auto& refNode = this->mPoseGraph->ScanNodes().at(
            loopCandidate.mReferenceScanNodeId);
        /* Retrieve the reference to the reference local grid map */
        const auto& refLocalMap = this->mGridMapBuilder->LocalMapAt(
            loopCandidate.mReferenceLocalMapId);
        /* Retrieve the reference to the reference local map node */
        const auto& refLocalMapNode = this->mPoseGraph->LocalMapNodes().at(
            loopCandidate.mReferenceLocalMapId);

        /* Append the data needed for loop detection */
        loopDetectionQueries.emplace_back(
            queryNode, refNode, refLocalMap, refLocalMapNode);
    }

    return loopDetectionQueries;
}

/* Append a first node with an associated scan data, and update the
 * current local grid map and the latest map */
bool LidarGraphSlam::AppendFirstNodeAndEdge(
    const RobotPose2D<double>& initialScanPose,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Use a diagonal matrix with sufficiently small values as a covariance
     * matrix to fix the relative pose of the first local map and the first
     * scan node to zero */
    const Eigen::Matrix3d covarianceMatrix =
        Eigen::DiagonalMatrix<double, 3>(1e-9, 1e-9, 1e-9);
    /* Append a new scan data and create a new pose and an edge */
    const bool localMapInserted = this->mGridMapBuilder->AppendScan(
        this->mPoseGraph, initialScanPose,
        covarianceMatrix, scanData);
    /* Return whether the new local map is created */
    return localMapInserted;
}

/* Append a new pose graph node and an odometry edge, and update the
 * current local grid map and the latest map */
bool LidarGraphSlam::AppendNodeAndEdge(
    const RobotPose2D<double>& relativeScanPose,
    const Eigen::Matrix3d& scanPoseCovarianceMatrix,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Append a new scan data and create a new pose and an edge */
    const bool localMapInserted = this->mGridMapBuilder->AppendScan(
        this->mPoseGraph, relativeScanPose,
        scanPoseCovarianceMatrix, scanData);
    /* Return whether the new local map is created */
    return localMapInserted;
}

/* Append new loop closing edges */
void LidarGraphSlam::AppendLoopClosingEdges(
    const LoopDetectionResultVector& loopDetectionResults)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* The number of the new loop closing constraints */
    std::size_t numOfNewLoopEdges = 0;

    /* Append new loop closing edges for each loop detection result */
    for (const auto& loopDetectionResult : loopDetectionResults) {
        /* Setup the pose graph edge parameters */
        /* The angular component in the pose is normalized beforehand */
        const RobotPose2D<double> relativePose =
            NormalizeAngle(loopDetectionResult.mRelativePose);

        /* Covariance matrix represents the uncertainty of the scan matching */
        /* Covariance matrix should not be rotated since it already represents
         * the covariance of the pose in the map-local coordinate frame and
         * the pose of the local grid map is not changed during the loop
         * detection (SLAM frontend just appends new nodes or edges and does
         * not modify the already existing ones) */

        /* Compute a information matrix by inverting a covariance matrix */
        const Eigen::Matrix3d informationMat =
            loopDetectionResult.mEstimatedCovMat.inverse();

        /* Retrieve the local map information */
        const auto& localMap = this->mGridMapBuilder->LocalMapAt(
            loopDetectionResult.mLocalMapNodeId);
        /* Make sure that the local map is in finished state */
        Assert(localMap.mFinished);
        /* Make sure that the we are creating an edge that represents an
         * inter-local grid map constraint, that is, the local map
         * `localMap` does not contain the scan node `mScanNodeId` */
        Assert(loopDetectionResult.mScanNodeId < localMap.mScanNodeIdMin ||
               loopDetectionResult.mScanNodeId > localMap.mScanNodeIdMax);

        /* Append a new loop closing constraint */
        this->mPoseGraph->Edges().emplace_back(
            loopDetectionResult.mLocalMapNodeId,
            loopDetectionResult.mScanNodeId,
            EdgeType::InterLocalMap, ConstraintType::Loop,
            relativePose, informationMat);
        /* Update the number of the new loop closing constraint */
        numOfNewLoopEdges++;
    }

    /* Update the metrics */
    this->mMetrics.mNumOfNewLoopEdges->Observe(numOfNewLoopEdges);
}

/* Update pose graph nodes and rebuild grid maps after loop closure */
void LidarGraphSlam::AfterLoopClosure(
    const std::vector<LocalMapId>& localMapIds,
    const std::vector<Eigen::Vector3d>& localMapPoses,
    const std::vector<NodeId>& scanNodeIds,
    const std::vector<Eigen::Vector3d>& scanPoses)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    Assert(localMapIds.size() == localMapPoses.size());
    Assert(scanNodeIds.size() == scanPoses.size());

    const std::size_t numOfOptimizedMapNodes = localMapPoses.size();
    const std::size_t numOfOptimizedScanNodes = scanPoses.size();

    /* Update local map nodes using the results from the pose graph
     * optimization (edges are not updated since they are constants) */
    for (std::size_t i = 0; i < numOfOptimizedMapNodes; ++i) {
        /* Retrieve the old local map node */
        auto& oldLocalMapNode =
            this->mPoseGraph->LocalMapNodes().at(localMapIds[i]);
        /* Update the node pose */
        oldLocalMapNode.mGlobalPose.mX = localMapPoses[i][0];
        oldLocalMapNode.mGlobalPose.mY = localMapPoses[i][1];
        oldLocalMapNode.mGlobalPose.mTheta = localMapPoses[i][2];
    }

    /* Update the scan nodes using the results from the pose graph */
    for (std::size_t i = 0; i < numOfOptimizedScanNodes; ++i) {
        /* Retrieve the old scan node */
        auto& oldScanNode = this->mPoseGraph->ScanNodes().at(scanNodeIds[i]);
        /* Update the node pose */
        oldScanNode.mGlobalPose.mX = scanPoses[i][0];
        oldScanNode.mGlobalPose.mY = scanPoses[i][1];
        oldScanNode.mGlobalPose.mTheta = scanPoses[i][2];
    }

    /* Retrieve the pose and Id of the last node which got updated by the
     * pose graph optimization */
    const LocalMapId lastLocalMapId = *std::max_element(
        localMapIds.cbegin(), localMapIds.cend());
    const NodeId lastScanNodeId = *std::max_element(
        scanNodeIds.cbegin(), scanNodeIds.cend());

    /* Retrieve the corresponding local map and scan node */
    const auto& lastScanNode =
        this->mPoseGraph->ScanNodes().at(lastScanNodeId);
    const auto& lastLocalMap =
        this->mGridMapBuilder->LocalMapAt(lastLocalMapId);

    /* Check that only finished local grid maps are optimized */
    /* We just check the last optimized local map `lastLocalMap`, local maps
     * older than this map should be finished */
    Assert(lastLocalMap.mFinished);
    /* The following predicates indicate that the last optimized scan node
     * `lastScanNode` belongs to the last optimized local map `lastLocalMap`
     * and is the last scan of the `lastLocalMap`, which means that no scan
     * that belongs to the unoptimized local grid map got optimized */
    Assert(lastScanNode.mLocalMapId == lastLocalMap.mId);
    Assert(lastScanNode.mNodeId == lastLocalMap.mScanNodeIdMax);

    /* Retrieve the first odometry edge that did not take part in the last
     * loop detection and pose graph optimization, which is likely to be
     * an inter-local grid map odometry constraint */
    const auto odomEdgeIt = std::find_if(
        this->mPoseGraph->Edges().cbegin(),
        this->mPoseGraph->Edges().cend(),
        [&lastLocalMap](const PoseGraphEdge& edge) {
            return edge.mLocalMapNodeId == lastLocalMap.mId &&
                   edge.mScanNodeId > lastLocalMap.mScanNodeIdMax; });

    /* If the above edge is not found, then no new node has been added
     * to the pose graph since the beginning of the loop closure and
     * we do not need to correct the remaining part of the pose graph */
    if (odomEdgeIt == this->mPoseGraph->Edges().cend()) {
        this->mGridMapBuilder->AfterLoopClosure(this->mPoseGraph);
        return;
    }

    /* Make sure that the above edge is odometry constraint */
    Assert(odomEdgeIt->IsOdometryConstraint());

    /* Assume that all odometry edges that are not considered in the last
     * loop closure are found in the pose graph edges after the above edge */
    LocalMapId processedLocalMapId = lastLocalMap.mId;
    NodeId processedNodeId = lastLocalMap.mScanNodeIdMax;

    /* Update remaining pose graph nodes after the above odometry edge,
     * which are added after the beginning of this loop closure,
     * using the relative poses extracted from the odometry edges */
    /* Relative poses between local map nodes and scan nodes inside these
     * local maps are kept */
    const Range odomEdges { odomEdgeIt, this->mPoseGraph->Edges().cend() };

    for (const auto& odomEdge : odomEdges) {
        /* Ignore if the edge represents the loop constraint */
        if (!odomEdge.IsOdometryConstraint())
            continue;

        /* Check what kind of node (local map or scan) should be updated */
        const bool updateScanNode =
            odomEdge.mLocalMapNodeId == processedLocalMapId &&
            odomEdge.mScanNodeId > processedNodeId;
        const bool updateLocalMapNode =
            odomEdge.mLocalMapNodeId > processedLocalMapId &&
            odomEdge.mScanNodeId == processedNodeId;

        /* Since the edge is not a loop constraint, either the scan node or
         * the local map node should be updated */
        Assert(updateScanNode || updateLocalMapNode);

        if (updateScanNode) {
            /* Retrieve the starting node whose pose is already updated */
            const LocalMapNode& startNode =
                this->mPoseGraph->LocalMapNodes().at(odomEdge.mLocalMapNodeId);
            /* Compute the new node pose */
            const RobotPose2D<double>& startNodePose = startNode.mGlobalPose;
            const RobotPose2D<double> endNodePose =
                Compound(startNodePose, odomEdge.mRelativePose);
            /* Update the node pose */
            ScanNode& endNode =
                this->mPoseGraph->ScanNodes().at(odomEdge.mScanNodeId);
            endNode.mGlobalPose = endNodePose;
        } else if (updateLocalMapNode) {
            /* Retrieve the ending node whose pose is already updated */
            const ScanNode& endNode =
                this->mPoseGraph->ScanNodes().at(odomEdge.mScanNodeId);
            /* Compute the new node pose */
            const RobotPose2D<double>& endNodePose = endNode.mGlobalPose;
            const RobotPose2D<double> startNodePose =
                MoveBackward(endNodePose, odomEdge.mRelativePose);
            /* Update the node pose */
            LocalMapNode& startNode =
                this->mPoseGraph->LocalMapNodes().at(odomEdge.mLocalMapNodeId);
            startNode.mGlobalPose = startNodePose;
        }

        /* Update the last processed edge information */
        processedLocalMapId = odomEdge.mLocalMapNodeId;
        processedNodeId = odomEdge.mScanNodeId;
    }

    /* Rebuild the grid maps */
    this->mGridMapBuilder->AfterLoopClosure(this->mPoseGraph);

    return;
}

/* Retrieve a latest map that contains latest scans */
void LidarGraphSlam::GetLatestMap(
    RobotPose2D<double>& globalPose,
    GridMap& latestMap,
    NodeId& scanNodeIdMin,
    NodeId& scanNodeIdMax) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Set the pose of the latest map in a world coordinate frame */
    globalPose = this->mGridMapBuilder->LatestMapPose();
    /* Set the latest grid map */
    latestMap = this->mGridMapBuilder->LatestMap();

    /* Set the Id range of the scan node */
    /* Scan data in the scan nodes within the range of `scanNodeIdMin` and
     * `scanNodeIdMax` are used to create a latest grid map */
    scanNodeIdMin = this->mGridMapBuilder->LatestScanIdMin();
    scanNodeIdMax = this->mGridMapBuilder->LatestScanIdMax();

    return;
}

/* Build a global map that contains all local grid maps acquired */
void LidarGraphSlam::GetGlobalMap(
    RobotPose2D<double>& globalPose,
    GridMap& globalMap,
    const NodeId scanNodeIdMin,
    const NodeId scanNodeIdMax) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Return a new global map and its pose in a world coordinate frame */
    this->mGridMapBuilder->ConstructGlobalMap(
        this->mPoseGraph, scanNodeIdMin, scanNodeIdMax, globalPose, globalMap);
}

/* Retrieve a collection of local grid maps */
void LidarGraphSlam::GetLocalMaps(
    IdMap<LocalMapId, LocalMap>& localMaps)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Return a collection of local grid maps */
    localMaps = this->mGridMapBuilder->LocalMaps();
}

/* Start the SLAM backend */
void LidarGraphSlam::StartBackend()
{
    /* Make sure that the thread is not created */
    assert(this->mBackendThread == nullptr);

    /* Create the worker thread for the SLAM backend */
    this->mBackendThread = std::make_shared<std::thread>(
        [this]() { this->mBackend->Run(
            this, std::ref(this->mBackendStopRequest)); });
}

/* Stop the SLAM backend */
void LidarGraphSlam::StopBackend()
{
    if (this->mBackendThread == nullptr)
        return;

    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Mark the current local grid map as finished */
    this->mGridMapBuilder->FinishLocalMap(this->mPoseGraph);

    /* Notify the SLAM backend to terminate */
    this->mBackendNotify = true;
    this->mBackendStopRequest = true;
    this->mBackendNotifyCond.notify_all();

    /* Release the lock */
    uniqueLock.unlock();

    /* Wait for the SLAM backend to terminate */
    if (this->mBackendThread->joinable())
        this->mBackendThread->join();

    /* Destroy the worker thread */
    this->mBackendThread.reset();
}

/* Notify the SLAM backend */
void LidarGraphSlam::NotifyBackend()
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Notify the SLAM backend */
    this->mBackendNotify = true;
    this->mBackendNotifyCond.notify_all();
}

/* Wait for the notification from the SLAM frontend */
void LidarGraphSlam::WaitForNotification()
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Wait for the notification from the SLAM frontend */
    this->mBackendNotifyCond.wait(uniqueLock,
        [this]() { return this->mBackendNotify; });
    /* Update the flag for notification */
    this->mBackendNotify = false;
}

/* Start the SLAM backend optimization */
void LidarGraphSlam::NotifyOptimizationStarted()
{
    /* Acquire the unique lock */
    std::unique_lock lock { this->mMutex };
    /* Set the flag to indicate that the SLAM backend has started */
    this->mOptimizationRunning = true;
}

/* Notify that the SLAM backend optimization is done */
void LidarGraphSlam::NotifyOptimizationDone()
{
    /* Acquire the unique lock */
    std::unique_lock lock { this->mMutex };
    /* Notify that the SLAM backend optimization is done, so that
     * the SLAM frontend can process the sensor input */
    this->mOptimizationRunning = false;
    this->mOptimizationDoneCond.notify_all();
}

/* Wait for the SLAM backend optimization to be done */
void LidarGraphSlam::WaitForOptimization()
{
    /* Acquire the unique lock */
    std::unique_lock lock { this->mMutex };
    /* Wait for the notification from the SLAM backend */
    this->mOptimizationDoneCond.wait(lock,
        [this]() { return !this->mOptimizationRunning; });
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
