
/* loop_detector_fpga_parallel.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_FPGA_PARALLEL_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_FPGA_PARALLEL_HPP

#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_correlative_fpga.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_correlative_fpga.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopDetectorFPGAParallel final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorFPGAParallel(
        const std::string& loopDetectorName,
        const std::shared_ptr<ScanMatcherCorrelativeFPGA>& scanMatcher0,
        const std::shared_ptr<ScanMatcherCorrelativeFPGA>& scanMatcher1,
        const std::shared_ptr<ScanMatcher>& finalScanMatcher0,
        const std::shared_ptr<ScanMatcher>& finalScanMatcher1,
        const double scoreThreshold,
        const double knownRateThreshold);

    /* Destructor */
    ~LoopDetectorFPGAParallel() = default;

    /* Find a loop and return a loop constraint */
    LoopDetectionResultVector Detect(
        const LoopDetectionQueryVector& queries) override;

private:
    /* Perform a loop detection given a query */
    void Detect(const int coreId,
                const std::size_t queryBeginIdx,
                const std::size_t queryEndIdx,
                const LoopDetectionQueryVector& queries,
                LoopDetectionResultVector& results,
                std::vector<int>& times);

private:
    /* Total number of the real-time correlative-based scan matcher
     * IP cores implemented on the Zynq device */
    static constexpr const int        NumOfIPCores = 2;

    /* Real-time correlative scan matcher (Hardware accelerator) */
    std::shared_ptr<ScanMatcherCorrelativeFPGA> mScanMatcher[NumOfIPCores];
    /* Final scan matcher that performs the scan-matching at sub-pixel
     * accuracy and refines the estimation results returned from the
     * correlative scan matcher */
    std::shared_ptr<ScanMatcher> mFinalScanMatcher[NumOfIPCores];
    /* Normalized matching score threshold for loop detection */
    const double mScoreThreshold;
    /* Threshold for the ratio of the known grid cells */
    const double mKnownRateThreshold;
    /* Metrics information */
    LoopDetectorFPGAMetrics mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_FPGA_PARALLEL_HPP */
