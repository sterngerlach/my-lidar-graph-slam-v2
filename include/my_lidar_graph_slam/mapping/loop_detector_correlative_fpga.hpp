
/* loop_detector_correlative_fpga.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_CORRELATIVE_FPGA_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_CORRELATIVE_FPGA_HPP

#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_correlative_fpga.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct LoopDetectorFPGAMetrics
{
    /* Constructor */
    LoopDetectorFPGAMetrics(const std::string& loopDetectorName);
    /* Destructor */
    ~LoopDetectorFPGAMetrics() = default;

    /* Total processing time for loop detection */
    Metric::DistributionBase*  mLoopDetectionTime;
    /* Number of the loop detection queries */
    Metric::ValueSequenceBase* mNumOfQueries;
    /* Number of the successful loop detections */
    Metric::ValueSequenceBase* mNumOfDetections;
};

class LoopDetectorCorrelativeFPGA final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorCorrelativeFPGA(
        const std::string& loopDetectorName,
        const std::shared_ptr<ScanMatcherCorrelativeFPGA>& scanMatcher,
        const std::shared_ptr<ScanMatcher>& finalScanMatcher,
        const double scoreThreshold,
        const double knownRateThreshold);

    /* Destructor */
    ~LoopDetectorCorrelativeFPGA() = default;

    /* Find a loop and return a loop constraint */
    LoopDetectionResultVector Detect(
        const LoopDetectionQueryVector& loopDetectionQueries) override;

private:
    /* Find a corresponding pose of the current robot pose
     * from a local grid map */
    bool FindCorrespondingPose(
        const GridMapType& localMap,
        const LocalMapId localMapId,
        const Point2D<double>& localMapCenterPos,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalScanPose,
        RobotPose2D<double>& correspondingPose,
        Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Real-time correlative scan matcher (Hardware accelerator) */
    std::shared_ptr<ScanMatcherCorrelativeFPGA> mScanMatcher;
    /* Final scan matcher that performs the scan-matching at sub-pixel
     * accuracy and refines the estimation results returned from the
     * correlative scan matcher */
    std::shared_ptr<ScanMatcher>                mFinalScanMatcher;
    /* Normalized matching score threshold for loop detection */
    const double                                mScoreThreshold;
    /* Threshold for the ratio of the known grid cells */
    const double                                mKnownRateThreshold;
    /* Metrics information */
    LoopDetectorFPGAMetrics                     mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_CORRELATIVE_FPGA_HPP */
