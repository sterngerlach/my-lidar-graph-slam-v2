
/* loop_detector_correlative.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_CORRELATIVE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_CORRELATIVE_HPP

#include <memory>
#include <vector>

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id_map.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_correlative.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct LoopDetectorCorrelativeMetrics
{
    /* Constructor */
    LoopDetectorCorrelativeMetrics(const std::string& loopDetectorName);
    /* Destructor */
    ~LoopDetectorCorrelativeMetrics() = default;

    /* Total processing time for grid map precomputations */
    Metric::ValueSequenceBase<int>* mInputSetupTime;
    /* Total processing time for loop detection */
    Metric::ValueSequenceBase<int>* mLoopDetectionTime;
    /* Number of the loop detection queries */
    Metric::ValueSequenceBase<int>* mNumOfQueries;
    /* Number of the successful loop detections */
    Metric::ValueSequenceBase<int>* mNumOfDetections;
};

class LoopDetectorCorrelative final : public LoopDetector
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
                            ConstMap&& precompMap) :
            mId(localMapId), mMap(std::move(precompMap)) { }

        /* Id of the local grid map */
        const LocalMapId mId;
        /* Precomputed coarser grid maps */
        const ConstMap   mMap;
    };

    /* Constructor */
    LoopDetectorCorrelative(
        const std::string& loopDetectorName,
        const std::shared_ptr<ScanMatcherCorrelative>& scanMatcher,
        const std::shared_ptr<ScanMatcher>& finalScanMatcher,
        const double scoreThreshold,
        const double knownRateThreshold);

    /* Destructor */
    ~LoopDetectorCorrelative() = default;

    /* Find a loop and return a loop constraint */
    LoopDetectionResultVector Detect(
        const LoopDetectionQueryVector& loopDetectionQueries) override;

private:
    /* Find a corresponding pose of the current robot pose
     * from a local grid map */
    bool FindCorrespondingPose(
        const GridMap& localMap,
        const ConstMap& precompMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalScanPose,
        RobotPose2D<double>& correspondingPose,
        Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Real-time correlative scan matcher */
    std::shared_ptr<ScanMatcherCorrelative> mScanMatcher;
    /* Final scan matcher that performs the scan-matching at sub-pixel
     * accuracy and refines the estimation results returned from the
     * correlative scan matcher */
    std::shared_ptr<ScanMatcher>            mFinalScanMatcher;
    /* Normalized matching score threshold for loop detection */
    const double                            mScoreThreshold;
    /* Threshold for the ratio of the known grid cells */
    const double                            mKnownRateThreshold;
    /* Precomputed grid maps for each local grid map */
    IdMap<LocalMapId, PrecomputedMapStack>  mPrecompMaps;
    /* Metrics information */
    LoopDetectorCorrelativeMetrics          mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_CORRELATIVE_HPP */
