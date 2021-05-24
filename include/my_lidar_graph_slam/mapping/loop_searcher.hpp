
/* loop_searcher.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_HPP

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id_map.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LoopSearchHint struct holds the necessary information
 * for searching the local grid map and the scan node, which are used
 * for the loop detection based on exhaustive scan matching
 */
struct LoopSearchHint final
{
    /* Constructor */
    LoopSearchHint(
        IdMap<NodeId, ScanNodeData>&& scanNodes,
        IdMap<LocalMapId, LocalMapData>&& localMapNodes,
        const double accumTravelDist,
        const NodeId lastFinishedScanId,
        const LocalMapId lastFinishedMapId) :
        mScanNodes(std::move(scanNodes)),
        mLocalMapNodes(std::move(localMapNodes)),
        mAccumTravelDist(accumTravelDist),
        mLastFinishedScanId(lastFinishedScanId),
        mLastFinishedMapId(lastFinishedMapId) { }

    /* Destructor */
    ~LoopSearchHint() = default;

    /* Information about the finished scan nodes */
    const IdMap<NodeId, ScanNodeData>     mScanNodes;
    /* Information about the finished local map nodes */
    const IdMap<LocalMapId, LocalMapData> mLocalMapNodes;
    /* Accumulated travel distance of the robot (current scan node) */
    const double                          mAccumTravelDist;
    /* Id of the scan node in the last finished local map */
    const NodeId                          mLastFinishedScanId;
    /* Id of the last finished local map */
    const LocalMapId                      mLastFinishedMapId;
};

/*
 * LoopCandidate struct holds a query scan node, a reference scan node, and
 * a reference local map node, which are used for the loop detection.
 * A scan data associated to the query node is matched against a scan data
 * associated to the reference node, or against a local map created from
 * multiple consecutive scans corresponding to scan nodes including the
 * reference node.
 */
struct LoopCandidate final
{
    /* Constructor */
    LoopCandidate(const NodeId queryScanNodeId,
                  const NodeId referenceScanNodeId,
                  const LocalMapId referenceLocalMapId) :
        mQueryScanNodeId(queryScanNodeId),
        mReferenceScanNodeId(referenceScanNodeId),
        mReferenceLocalMapId(referenceLocalMapId) { }

    /* Destructor */
    ~LoopCandidate() = default;

    /* Id of the query scan node */
    const NodeId     mQueryScanNodeId;
    /* Id of the reference scan node */
    const NodeId     mReferenceScanNodeId;
    /* Id of the reference local map node */
    const LocalMapId mReferenceLocalMapId;
};

/* Vector of the loop candidates */
using LoopCandidateVector = std::vector<LoopCandidate>;

class LoopSearcher
{
public:
    /* Constructor */
    LoopSearcher() = default;

    /* Destructor */
    virtual ~LoopSearcher() = default;

    /* Find a local map and a scan node for loop detection */
    virtual LoopCandidateVector Search(
        const LoopSearchHint& searchHint) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_HPP */
