
/* loop_detector_branch_bound.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_BRANCH_BOUND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_BRANCH_BOUND_HPP

#include <map>
#include <stack>
#include <vector>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id_map.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_branch_bound.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct LoopDetectorBranchBoundMetrics
{
    /* Constructor */
    LoopDetectorBranchBoundMetrics(const std::string& loopDetectorTime);
    /* Destructor */
    ~LoopDetectorBranchBoundMetrics() = default;

    /* Total processing time for grid map precomputations */
    Metric::ValueSequenceBase<int>*           mInputSetupTime;
    /* Total processing time for loop detection */
    Metric::ValueSequenceBase<int>*           mLoopDetectionTime;
    /* Number of the loop detection queries */
    Metric::ValueSequenceBase<int>*           mNumOfQueries;
    /* Number of the successful loop detections */
    Metric::ValueSequenceBase<int>*           mNumOfDetections;
    /* Total memory consumption for the precomputed grid maps */
    Metric::ValueSequenceBase<std::uint64_t>* mPrecompMapMemoryUsage;
};

class LoopDetectorBranchBound final : public LoopDetector
{
public:
    /*
     * PrecomputedMapStack struct holds the precomputed coarser
     * (low-resolution) grid maps for each local grid map
     */
    struct PrecomputedMapStack
    {
        /* Constructor */
        PrecomputedMapStack(const LocalMapId& localMapId,
                            std::vector<ConstMap>&& precompMaps) :
            mId(localMapId), mMaps(std::move(precompMaps)) { }

        /* Inspect the memory usage in bytes */
        inline std::uint64_t InspectMemoryUsage() const {
            const std::uint64_t mapMemoryUsage = std::accumulate(
                this->mMaps.begin(), this->mMaps.end(), 0,
                [](const std::uint64_t memoryUsage, const ConstMap& constMap) {
                    return memoryUsage + constMap.InspectMemoryUsage(); });
            return sizeof(this->mId) + sizeof(this->mMaps) + mapMemoryUsage; }

        /* Id of the local grid map */
        const LocalMapId            mId;
        /* Precomputed coarser grid maps */
        const std::vector<ConstMap> mMaps;
    };

    /* Constructor */
    LoopDetectorBranchBound(
        const std::string& loopDetectorName,
        const std::shared_ptr<ScanMatcherBranchBound>& scanMatcher,
        const std::shared_ptr<ScanMatcher>& finalScanMatcher,
        const double scoreThreshold,
        const double knownRateThreshold);

    /* Destructor */
    ~LoopDetectorBranchBound() = default;

    /* Find a loop and return a loop constraint */
    LoopDetectionResultVector Detect(
        const LoopDetectionQueryVector& queries) override;

private:
    /* Branch-and-bound based scan matcher */
    std::shared_ptr<ScanMatcherBranchBound> mScanMatcher;
    /* Final scan matcher that performs the scan-matching at sub-pixel
     * accuracy and refines the estimation results returned from the
     * correlative scan matcher */
    std::shared_ptr<ScanMatcher>            mFinalScanMatcher;
    /* Normalized matching score threshold for loop detector */
    const double                            mScoreThreshold;
    /* Threshold for the ratio of the known grid cells */
    const double                            mKnownRateThreshold;
    /* Precomputed grid maps for each local grid map */
    IdMap<LocalMapId, PrecomputedMapStack>  mPrecompMaps;
    /* Metrics information */
    LoopDetectorBranchBoundMetrics          mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_BRANCH_BOUND_HPP */
