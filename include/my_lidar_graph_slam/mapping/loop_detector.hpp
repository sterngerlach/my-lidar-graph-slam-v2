
/* loop_detector.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_HPP

#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LoopDetectionQuery struct holds the detailed information for
 * a loop detection, including a collection of nodes and a local grid map.
 * Each node contains its pose in a world frame and an associated scan data
 */
struct LoopDetectionQuery final
{
    /* Constructor */
    LoopDetectionQuery(const ScanNode& queryScanNode,
                       const ScanNode& referenceScanNode,
                       const LocalMap& referenceLocalMap,
                       const LocalMapNode& referenceLocalMapNode) :
        mQueryScanNode(queryScanNode),
        mReferenceScanNode(referenceScanNode),
        mReferenceLocalMap(referenceLocalMap),
        mReferenceLocalMapNode(referenceLocalMapNode) { }

    /* Destructor */
    ~LoopDetectionQuery() = default;

    /* Query scan node, which is matched against the reference local grid map
     * using the node pose and its associated scan data */
    const ScanNode&       mQueryScanNode;
    /* Reference scan node, which resides in the reference local map, and
     * is the closest scan node to the query scan node */
    const ScanNode&       mReferenceScanNode;
    /* Reference local grid map, which has the reference data type,
     * since it is in a finished state and is not updated anymore */
    const LocalMap&       mReferenceLocalMap;
    /* Reference local map node, which has the reference data type,
     * since it is not modified by the SLAM frontend which constructs
     * the pose graph but does not modify the existing pose graph nodes */
    const LocalMapNode&   mReferenceLocalMapNode;
};

/* Vector of the loop detection query */
using LoopDetectionQueryVector = std::vector<LoopDetectionQuery>;

/*
 * LoopDetectionResult struct holds the result for a loop detection,
 * necessary information for constructing a pose graph edge that represents
 * a loop closing constraint
 */
struct LoopDetectionResult final
{
    /* Constructor */
    LoopDetectionResult(const RobotPose2D<double>& relativePose,
                        const RobotPose2D<double>& localMapPose,
                        const LocalMapId localMapNodeId,
                        const NodeId scanNodeId,
                        const Eigen::Matrix3d& estimatedCovMat) :
        mRelativePose(relativePose),
        mLocalMapPose(localMapPose),
        mLocalMapNodeId(localMapNodeId),
        mScanNodeId(scanNodeId),
        mEstimatedCovMat(estimatedCovMat) { }

    /* Destructor */
    ~LoopDetectionResult() = default;

    /* Actual relative pose between two nodes */
    const RobotPose2D<double> mRelativePose;
    /* Pose of the start node (local grid map) in a global frame */
    const RobotPose2D<double> mLocalMapPose;
    /* Id of the local map node */
    const LocalMapId          mLocalMapNodeId;
    /* Id of the scan node */
    const NodeId              mScanNodeId;
    /* Estimated covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d     mEstimatedCovMat;
};

/* Vector of the loop detection results */
using LoopDetectionResultVector = std::vector<LoopDetectionResult>;

class LoopDetector
{
public:
    /* Constructor */
    LoopDetector(const std::string& loopDetectorName) :
        mName(loopDetectorName) { }
    /* Destructor */
    virtual ~LoopDetector() = default;

    /* Retrieve the name of this loop detector */
    inline const std::string& Name() const { return this->mName; }

    /* Find a loop and return a loop constraint */
    virtual LoopDetectionResultVector Detect(
        const LoopDetectionQueryVector& loopDetectionQueries) = 0;

protected:
    /* Name of this loop detector */
    const std::string mName;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_HPP */
