
/* loop_detector_grid_search.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_GRID_SEARCH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_GRID_SEARCH_HPP

#include <memory>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_grid_search.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct LoopDetectorGridSearchMetrics
{
    /* Constructor */
    LoopDetectorGridSearchMetrics(const std::string& loopDetectorName);
    /* Destructor */
    ~LoopDetectorGridSearchMetrics() = default;

    /* Total processing time for loop detection */
    Metric::DistributionBase*  mLoopDetectionTime;
    /* Number of the loop detection queries */
    Metric::ValueSequenceBase* mNumOfQueries;
    /* Number of the successful loop detections */
    Metric::ValueSequenceBase* mNumOfDetections;
};

class LoopDetectorGridSearch final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorGridSearch(
        const std::string& loopDetectorName,
        const std::shared_ptr<ScanMatcherGridSearch>& scanMatcher,
        const std::shared_ptr<ScanMatcher>& finalScanMatcher,
        const double scoreThreshold,
        const double knownRateThreshold);

    /* Destructor */
    ~LoopDetectorGridSearch() = default;

    /* Find a loop and return a loop constraint */
    LoopDetectionResultVector Detect(
        const LoopDetectionQueryVector& loopDetectionQueries) override;

private:
    /* Find a corresponding pose of the current robot pose
     * from the local grid map */
    bool FindCorrespondingPose(const GridMapType& gridMap,
                               const Sensor::ScanDataPtr<double>& scanData,
                               const RobotPose2D<double>& mapLocalScanPose,
                               RobotPose2D<double>& correspondingPose,
                               Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Exhaustive grid search based scan matcher */
    std::shared_ptr<ScanMatcherGridSearch> mScanMatcher;
    /* Final scan matcher that performs the scan-matching at sub-pixel
     * accuracy and refines the estimation results returned from the
     * correlative scan matcher */
    std::shared_ptr<ScanMatcher>           mFinalScanMatcher;
    /* Normalized matching score threshold for loop detector */
    const double                           mScoreThreshold;
    /* Threshold for the ratio of the known grid cells */
    const double                           mKnownRateThreshold;
    /* Metrics information */
    LoopDetectorGridSearchMetrics          mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_GRID_SEARCH_HPP */
