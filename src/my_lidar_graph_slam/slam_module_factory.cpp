
/* slam_module_factory.cpp */

#include "my_lidar_graph_slam/slam_module_factory.hpp"

#include "my_lidar_graph_slam/fpga_module_factory.hpp"
#include "my_lidar_graph_slam/loop_detector_factory.hpp"
#include "my_lidar_graph_slam/pose_graph_optimizer_factory.hpp"
#include "my_lidar_graph_slam/scan_filter_factory.hpp"
#include "my_lidar_graph_slam/scan_matcher_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Create a grid map builder */
std::shared_ptr<Mapping::GridMapBuilder> CreateGridMapBuilder(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for grid map builder */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double mapResolution = config.get<double>("Map.Resolution");
    const int patchSize = config.get<int>("Map.PatchSize");
    const int numOfLatestScans =
        config.get<int>("Map.NumOfScansForLatestMap");
    const double localMapTravelDist =
        config.get<double>("Map.TravelDistThresholdForLocalMap");
    const std::size_t numOfOverlappedScans =
        config.get<std::size_t>("Map.NumOfOverlappedScans");

    const double usableRangeMin = config.get<double>("UsableRangeMin");
    const double usableRangeMax = config.get<double>("UsableRangeMax");

    const double probHit = config.get<double>("ProbabilityHit");
    const double probMiss = config.get<double>("ProbabilityMiss");

    /* Construct grid map builder object */
    auto pGridMapBuilder = std::make_shared<Mapping::GridMapBuilder>(
        mapResolution, patchSize, numOfLatestScans, localMapTravelDist,
        numOfOverlappedScans, usableRangeMin, usableRangeMax,
        probHit, probMiss);

    return pGridMapBuilder;
}

/* Create a SLAM frontend */
std::shared_ptr<Mapping::LidarGraphSlamFrontend> CreateSlamFrontend(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for SLAM frontend */
    const auto& config = jsonSettings.get_child(configGroup);

    /* Create scan outlier filter if necessary */
    const bool useScanOutlierFilter =
        config.get<bool>("UseScanOutlierFilter");
    const std::string scanOutlierFilterConfigGroup =
        config.get<std::string>("ScanOutlierFilterConfigGroup");

    auto pScanOutlierFilter = useScanOutlierFilter ?
        CreateScanOutlierFilter(jsonSettings, scanOutlierFilterConfigGroup) :
        nullptr;

    /* Create scan accumulator if necessary */
    const bool useScanAccumulator =
        config.get<bool>("UseScanAccumulator");
    const std::string scanAccumulatorConfigGroup =
        config.get<std::string>("ScanAccumulatorConfigGroup");

    auto pScanAccumulator = useScanAccumulator ?
        CreateScanAccumulator(jsonSettings, scanAccumulatorConfigGroup) :
        nullptr;

    /* Create scan interpolator if necessary */
    const bool useScanInterpolator =
        config.get<bool>("UseScanInterpolator");
    const std::string scanInterpolatorConfigGroup =
        config.get<std::string>("ScanInterpolatorConfigGroup");

    auto pScanInterpolator = useScanInterpolator ?
        CreateScanInterpolator(jsonSettings, scanInterpolatorConfigGroup) :
        nullptr;

    /* Check if the scan matching is offloaded to the FPGA device */
    const bool useHardwareScanMatcher =
        config.get<bool>("LocalSlam.UseHardwareScanMatcher");

    Mapping::ScanMatcherPtr pScanMatcher = nullptr;

    if (useHardwareScanMatcher) {
        /* Create the FPGA-based scan matcher for local SLAM */
        pScanMatcher = CreateScanMatcherCorrelativeFPGA(
            jsonSettings, "Hardware.ScanMatcher");
    } else {
        /* Create the software scan matcher for local SLAM */
        const std::string localScanMatcherType =
            config.get<std::string>("LocalSlam.ScanMatcherType");
        const std::string localScanMatcherConfigGroup =
            config.get<std::string>("LocalSlam.ScanMatcherConfigGroup");
        pScanMatcher = CreateScanMatcher(
            jsonSettings, localScanMatcherType,
            localScanMatcherConfigGroup, "");
    }

    /* Create the final scan matcher for local SLAM */
    const std::string finalScanMatcherType =
        config.get<std::string>("LocalSlam.FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("LocalSlam.FinalScanMatcherConfigGroup");

    auto pFinalScanMatcher = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup, "");

    /* Load the initial pose */
    const double initialPoseX = config.get<double>("InitialPose.X");
    const double initialPoseY = config.get<double>("InitialPose.Y");
    const double initialPoseTheta = config.get<double>("InitialPose.Theta");

    const RobotPose2D<double> initialPose {
        initialPoseX, initialPoseY, initialPoseTheta };

    /* Load the threshold values for local SLAM */
    const double updateThresholdTravelDist =
        config.get<double>("UpdateThresholdTravelDist");
    const double updateThresholdAngle =
        config.get<double>("UpdateThresholdAngle");
    const double updateThresholdTime =
        config.get<double>("UpdateThresholdTime");

    /* Load settings for triggering loop detections */
    const double loopDetectionThreshold =
        config.get<double>("LoopDetectionThreshold");

    /* Load settings for checking degenerations */
    const double degenerationThreshold =
        config.get<double>("DegenerationThreshold");
    const double odometryCovarianceScale =
        config.get<double>("OdometryCovarianceScale");
    const bool fuseOdometryCovariance =
        config.get<bool>("FuseOdometryCovariance");

    /* Create SLAM frontend */
    auto pFrontend = std::make_shared<Mapping::LidarGraphSlamFrontend>(
        pScanOutlierFilter, pScanAccumulator, pScanInterpolator,
        pScanMatcher, pFinalScanMatcher,
        initialPose, updateThresholdTravelDist, updateThresholdAngle,
        updateThresholdTime, loopDetectionThreshold,
        degenerationThreshold, odometryCovarianceScale, fuseOdometryCovariance);

    return pFrontend;
}

/* Create a SLAM backend */
std::shared_ptr<Mapping::LidarGraphSlamBackend> CreateSlamBackend(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for SLAM backend */
    const auto& config = jsonSettings.get_child(configGroup);

    /* Create pose graph optimizer */
    const std::string optimizerType =
        config.get<std::string>("PoseGraphOptimizerType");
    const std::string optimizerConfigGroup =
        config.get<std::string>("PoseGraphOptimizerConfigGroup");
    auto pOptimizer = CreatePoseGraphOptimizer(
        jsonSettings, optimizerType, optimizerConfigGroup);

    /* Create loop searcher */
    const std::string loopSearcherType =
        config.get<std::string>("LoopSearcherType");
    const std::string loopSearcherConfigGroup =
        config.get<std::string>("LoopSearcherConfigGroup");
    auto pLoopSearcher = CreateLoopSearcher(
        jsonSettings, loopSearcherType, loopSearcherConfigGroup);

    /* Check if the loop detection is offloaded to the FPGA device */
    const bool useHardwareLoopDetector =
        config.get<bool>("UseHardwareLoopDetector");
    const bool useParallelLoopDetector =
        config.get<bool>("UseParallelLoopDetector");

    std::unique_ptr<Mapping::LoopDetector> pLoopDetector = nullptr;

    if (useHardwareLoopDetector) {
        /* Create the FPGA-based loop detector */
        if (!useParallelLoopDetector)
            pLoopDetector = CreateLoopDetectorCorrelativeFPGA(
                jsonSettings, "Hardware.LoopDetector");
        else
            pLoopDetector = CreateLoopDetectorFPGAParallel(
                jsonSettings, "Hardware.LoopDetectorParallel");
    } else {
        /* Create the software loop detector */
        const std::string loopDetectorType =
            config.get<std::string>("LoopDetectorType");
        const std::string loopDetectorConfigGroup =
            config.get<std::string>("LoopDetectorConfigGroup");
        pLoopDetector = CreateLoopDetector(
            jsonSettings, loopDetectorType, loopDetectorConfigGroup);
    }

    /* Create SLAM backend */
    auto pBackend = std::make_shared<Mapping::LidarGraphSlamBackend>(
        std::move(pOptimizer), std::move(pLoopSearcher),
        std::move(pLoopDetector));

    return pBackend;
}

/* Create a LiDAR Graph-Based SLAM module */
std::shared_ptr<Mapping::LidarGraphSlam> CreateLidarGraphSlam(
    const pt::ptree& jsonSettings)
{
    /* Load settings for LiDAR Graph-Based SLAM */
    const pt::ptree& config = jsonSettings.get_child("LidarGraphSlam");

    /* Create grid map builder */
    const std::string gridMapBuilderConfigGroup =
        config.get<std::string>("GridMapBuilderConfigGroup");
    auto pGridMapBuilder = CreateGridMapBuilder(
        jsonSettings, gridMapBuilderConfigGroup);

    /* Create pose graph */
    auto pPoseGraph = std::make_shared<Mapping::PoseGraph>();

    /* Create SLAM frontend object */
    const std::string frontendConfigGroup =
        config.get<std::string>("FrontendConfigGroup");
    auto pFrontend = CreateSlamFrontend(jsonSettings, frontendConfigGroup);

    /* Create SLAM backend object */
    const std::string backendConfigGroup =
        config.get<std::string>("BackendConfigGroup");
    auto pBackend = CreateSlamBackend(jsonSettings, backendConfigGroup);

    /* Create LiDAR Graph-Based SLAM object */
    auto pLidarGraphSlam = std::make_shared<Mapping::LidarGraphSlam>(
        pFrontend, pBackend, pGridMapBuilder, pPoseGraph);

    return pLidarGraphSlam;
}

} /* namespace MyLidarGraphSlam */
