
/* fpga_module_factory.cpp */

#include "my_lidar_graph_slam/fpga_module_factory.hpp"

#include "my_lidar_graph_slam/cost_function_factory.hpp"
#include "my_lidar_graph_slam/scan_matcher_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Load the register offsets for the real-time correlative-based
 * scan matcher IP core */
void LoadScanMatcherHardwareRegisterOffsets(
    const pt::ptree& jsonSettings,
    Mapping::ScanMatcherHardwareConfig& scanMatcherConfig)
{
    const auto getString = [&jsonSettings](const std::string& configName) {
        return jsonSettings.get<std::string>(configName); };
    const auto toAddress = [](const std::string& value) {
        return std::stoul(value, nullptr, 0); };

    /* Load settings for the register offsets */
    const auto offsetApCtrl = getString("AxiLiteSApCtrl");
    const auto offsetGIE = getString("AxiLiteSGIE");
    const auto offsetIER = getString("AxiLiteSIER");
    const auto offsetISR = getString("AxiLiteSISR");
    const auto offsetNumOfScans = getString("AxiLiteSNumOfScans");
    const auto offsetScanRangeMax = getString("AxiLiteSScanRangeMax");
    const auto offsetScoreThreshold = getString("AxiLiteSScoreThreshold");
    const auto offsetPoseX = getString("AxiLiteSPoseX");
    const auto offsetPoseY = getString("AxiLiteSPoseY");
    const auto offsetPoseTheta = getString("AxiLiteSPoseTheta");
    const auto offsetMapSizeX = getString("AxiLiteSMapSizeX");
    const auto offsetMapSizeY = getString("AxiLiteSMapSizeY");
    const auto offsetMapMinX = getString("AxiLiteSMapMinX");
    const auto offsetMapMinY = getString("AxiLiteSMapMinY");
    const auto offsetWinX = getString("AxiLiteSWinX");
    const auto offsetWinY = getString("AxiLiteSWinY");
    const auto offsetWinTheta = getString("AxiLiteSWinTheta");
    const auto offsetStepX = getString("AxiLiteSStepX");
    const auto offsetStepY = getString("AxiLiteSStepY");
    const auto offsetStepTheta = getString("AxiLiteSStepTheta");

    /* Set the register offsets */
    scanMatcherConfig.mAxiLiteSApCtrl = toAddress(offsetApCtrl);
    scanMatcherConfig.mAxiLiteSGIE = toAddress(offsetGIE);
    scanMatcherConfig.mAxiLiteSIER = toAddress(offsetIER);
    scanMatcherConfig.mAxiLiteSISR = toAddress(offsetISR);
    scanMatcherConfig.mAxiLiteSNumOfScans = toAddress(offsetNumOfScans);
    scanMatcherConfig.mAxiLiteSScanRangeMax = toAddress(offsetScanRangeMax);
    scanMatcherConfig.mAxiLiteSScoreThreshold = toAddress(offsetScoreThreshold);
    scanMatcherConfig.mAxiLiteSPoseX = toAddress(offsetPoseX);
    scanMatcherConfig.mAxiLiteSPoseY = toAddress(offsetPoseY);
    scanMatcherConfig.mAxiLiteSPoseTheta = toAddress(offsetPoseTheta);
    scanMatcherConfig.mAxiLiteSMapSizeX = toAddress(offsetMapSizeX);
    scanMatcherConfig.mAxiLiteSMapSizeY = toAddress(offsetMapSizeY);
    scanMatcherConfig.mAxiLiteSMapMinX = toAddress(offsetMapMinX);
    scanMatcherConfig.mAxiLiteSMapMinY = toAddress(offsetMapMinY);
    scanMatcherConfig.mAxiLiteSWinX = toAddress(offsetWinX);
    scanMatcherConfig.mAxiLiteSWinY = toAddress(offsetWinY);
    scanMatcherConfig.mAxiLiteSWinTheta = toAddress(offsetWinTheta);
    scanMatcherConfig.mAxiLiteSStepX = toAddress(offsetStepX);
    scanMatcherConfig.mAxiLiteSStepY = toAddress(offsetStepY);
    scanMatcherConfig.mAxiLiteSStepTheta = toAddress(offsetStepTheta);
}

/* Create the real-time correlative-based scan matcher IP core interface */
std::shared_ptr<Mapping::ScanMatcherCorrelativeFPGA>
    CreateScanMatcherCorrelativeFPGA(
        const pt::ptree& jsonSettings,
        const std::string& configGroup)
{
    /* Load settings for the real-time correlative scan matcher */
    Mapping::ScanMatcherHardwareConfig hardwareConfig;
    Mapping::AxiDmaConfig axiDmaConfig;

    /* Load the register offsets */
    const pt::ptree& commonSettings =
        jsonSettings.get_child("Hardware.Common");
    LoadScanMatcherHardwareRegisterOffsets(commonSettings, hardwareConfig);

    /* Load the hardware-specific parameters */
    const pt::ptree& scanMatcherSettings =
        jsonSettings.get_child(configGroup);

    const std::string scanMatcherName =
        scanMatcherSettings.get<std::string>("Name");
    const int maxNumOfScans =
        scanMatcherSettings.get<int>("MaxNumOfScans");
    const double mapResolution =
        scanMatcherSettings.get<double>("MapResolution");
    const int maxMapSizeX =
        scanMatcherSettings.get<int>("MaxMapSizeX");
    const int maxMapSizeY =
        scanMatcherSettings.get<int>("MaxMapSizeY");
    const int lowResolution =
        scanMatcherSettings.get<int>("CoarseMapResolution");
    const int mapBitWidth =
        scanMatcherSettings.get<int>("MapBitWidth");
    const int mapChunkWidth =
        scanMatcherSettings.get<int>("MapChunkWidth");

    const std::string axiLiteSBaseAddress =
        scanMatcherSettings.get<std::string>("AxiLiteSBaseAddress");
    const std::string axiLiteSAddressRange =
        scanMatcherSettings.get<std::string>("AxiLiteSAddressRange");
    const std::string axiDmaBaseAddress =
        scanMatcherSettings.get<std::string>("AxiDmaBaseAddress");
    const std::string axiDmaAddressRange =
        scanMatcherSettings.get<std::string>("AxiDmaAddressRange");

    hardwareConfig.mMaxNumOfScans = maxNumOfScans;
    hardwareConfig.mMapResolution = mapResolution;
    hardwareConfig.mMaxMapSizeX = maxMapSizeX;
    hardwareConfig.mMaxMapSizeY = maxMapSizeY;
    hardwareConfig.mLowResolution = lowResolution;
    hardwareConfig.mMapBitWidth = mapBitWidth;
    hardwareConfig.mMapChunkWidth = mapChunkWidth;

    hardwareConfig.mAxiLiteSBaseAddress = static_cast<std::uint32_t>(
        std::stoul(axiLiteSBaseAddress, nullptr, 0));
    hardwareConfig.mAxiLiteSAddressRange = static_cast<std::uint32_t>(
        std::stoul(axiLiteSAddressRange, nullptr, 0));

    axiDmaConfig.mBaseAddress = static_cast<std::uint32_t>(
        std::stoul(axiDmaBaseAddress, nullptr, 0));
    axiDmaConfig.mAddressRange = static_cast<std::uint32_t>(
        std::stoul(axiDmaAddressRange, nullptr, 0));

    /* Load the scan matching parameters */
    const double searchRangeX =
        scanMatcherSettings.get<double>("SearchRangeX");
    const double searchRangeY =
        scanMatcherSettings.get<double>("SearchRangeY");
    const double searchRangeTheta =
        scanMatcherSettings.get<double>("SearchRangeTheta");

    /* Create the cost function */
    const std::string costType =
        scanMatcherSettings.get<std::string>("CostType");
    const std::string costConfigGroup =
        scanMatcherSettings.get<std::string>("CostConfigGroup");
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Create the real-time correlative-based scan matcher IP core interface */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherCorrelativeFPGA>(
        scanMatcherName, pCostFunc,
        searchRangeX, searchRangeY, searchRangeTheta,
        std::move(hardwareConfig), std::move(axiDmaConfig));

    return pScanMatcher;
}

/* Create the real-time correlative-based loop detector IP core interface */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorCorrelativeFPGA(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for the real-time correlative loop detector */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::string loopDetectorName =
        config.get<std::string>("LoopDetectorName");
    const double scoreThreshold =
        config.get<double>("ScoreThreshold");
    const double knownRateThreshold =
        config.get<double>("KnownRateThreshold");

    const std::string scanMatcherConfigGroup =
        config.get<std::string>("ScanMatcherConfigGroup");
    const std::string finalScanMatcherType =
        config.get<std::string>("FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("FinalScanMatcherConfigGroup");

    /* Create a new real-time correlative scan matcher */
    auto pScanMatcher =  CreateScanMatcherCorrelativeFPGA(
        jsonSettings, scanMatcherConfigGroup);
    /* Construct a final scan matcher for refinements */
    auto pFinalScanMatcher = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup);

    /* Create a real-time correlative loop detector */
    auto pLoopDetector = std::make_unique<
        Mapping::LoopDetectorCorrelativeFPGA>(
            loopDetectorName, pScanMatcher, pFinalScanMatcher,
            scoreThreshold, knownRateThreshold);

    return pLoopDetector;
}

/* Create the real-time correlative-based loop detector which uses two IP cores
 * at the same time to improve the loop detection performance */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorFPGAParallel(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup)
{
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::string loopDetectorName =
        config.get<std::string>("LoopDetectorName");
    const double scoreThreshold =
        config.get<double>("ScoreThreshold");
    const double knownRateThreshold =
        config.get<double>("KnownRateThreshold");

    const std::string scanMatcher0ConfigGroup =
        config.get<std::string>("ScanMatcher0ConfigGroup");
    const std::string scanMatcher1ConfigGroup =
        config.get<std::string>("ScanMatcher1ConfigGroup");
    const std::string finalScanMatcherType =
        config.get<std::string>("FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("FinalScanMatcherConfigGroup");

    /* Create a new real-time correlative scan matcher */
    auto pScanMatcher0 = CreateScanMatcherCorrelativeFPGA(
        jsonSettings, scanMatcher0ConfigGroup);
    auto pScanMatcher1 = CreateScanMatcherCorrelativeFPGA(
        jsonSettings, scanMatcher1ConfigGroup);
    /* Create a final scan matcher for refinement */
    auto pFinalScanMatcher0 = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup);
    auto pFinalScanMatcher1 = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup);

    /* Create a real-time correlative-based loop detector */
    return std::make_unique<Mapping::LoopDetectorFPGAParallel>(
        loopDetectorName, pScanMatcher0, pScanMatcher1,
        pFinalScanMatcher0, pFinalScanMatcher1,
        scoreThreshold, knownRateThreshold);
}

} /* namespace MyLidarGraphSlam */
