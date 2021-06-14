
/* slam_launcher.cpp */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <vector>

#ifdef __GNUC__
#if (__GNUC__ >= 6) && (__GNUC__ < 8)
#include <experimental/filesystem>
#elif (__GNUC__ >= 8)
#include <filesystem>
#endif
#endif

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/hw/bitstream_loader.hpp"
#include "my_lidar_graph_slam/hw/cma_manager.hpp"
#include "my_lidar_graph_slam/io/gnuplot_helper.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/io/carmen/carmen_reader.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_greedy_endpoint.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_square_error.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_frontend.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_branch_bound.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_empty.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_grid_search.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_correlative.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_correlative_fpga.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher_nearest.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_g2o.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_lm.hpp"
#include "my_lidar_graph_slam/mapping/robust_loss_function.hpp"
#include "my_lidar_graph_slam/mapping/scan_accumulator.hpp"
#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_branch_bound.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_correlative.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_correlative_fpga.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_grid_search.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_hill_climbing.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_linear_solver.hpp"
#include "my_lidar_graph_slam/mapping/scan_outlier_filter.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

using namespace MyLidarGraphSlam;

/* Declare namespaces for convenience */
namespace pt = boost::property_tree;

#ifdef __GNUC__
#if (__GNUC__ >= 6) && (__GNUC__ < 8)
namespace fs = std::experimental::filesystem;
#elif (__GNUC__ >= 8)
namespace fs = std::filesystem;
#endif
#endif

/* Create the greedy endpoint cost function object */
std::shared_ptr<Mapping::CostFunction> CreateCostGreedyEndpoint(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for greedy endpoint cost function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double mapResolution = config.get<double>("MapResolution");
    const double hitAndMissedDist = config.get<double>("HitAndMissedDist");
    const double occupancyThreshold = config.get<double>("OccupancyThreshold");
    const int kernelSize = config.get<int>("KernelSize");
    const double standardDeviation = config.get<double>("StandardDeviation");
    const double scalingFactor = config.get<double>("ScalingFactor");

    /* Create greedy endpoint cost function */
    auto pCostFunc = std::make_shared<Mapping::CostGreedyEndpoint>(
        mapResolution, hitAndMissedDist,
        occupancyThreshold, kernelSize, standardDeviation, scalingFactor);

    return pCostFunc;
}

/* Create the square error cost function object */
std::shared_ptr<Mapping::CostFunction> CreateCostSquareError(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for square error cost function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double covarianceScale = config.get<double>("CovarianceScale");
    
    /* Create cost function object */
    auto pCostFunc = std::make_shared<Mapping::CostSquareError>(
        covarianceScale);
    
    return pCostFunc;
}

/* Create the cost function object */
std::shared_ptr<Mapping::CostFunction> CreateCostFunction(
    const pt::ptree& jsonSettings,
    const std::string& costType,
    const std::string& configGroup)
{
    if (costType == "GreedyEndpoint")
        return CreateCostGreedyEndpoint(jsonSettings, configGroup);
    else if (costType == "SquareError")
        return CreateCostSquareError(jsonSettings, configGroup);
    
    return nullptr;
}

/* Create the pixel-accurate score function object */
std::shared_ptr<Mapping::ScoreFunction> CreateScorePixelAccurate(
    const pt::ptree& /* jsonSettings */,
    const std::string& /* configGroup */)
{
    /* Create score function object */
    auto pScoreFunc = std::make_shared<Mapping::ScorePixelAccurate>();
    return pScoreFunc;
}

/* Create the score function object */
std::shared_ptr<Mapping::ScoreFunction> CreateScoreFunction(
    const pt::ptree& jsonSettings,
    const std::string& scoreType,
    const std::string& configGroup)
{
    if (scoreType == "PixelAccurate")
        return CreateScorePixelAccurate(jsonSettings, configGroup);
    
    return nullptr;
}

/* Create a new branch-and-bound based scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherBranchBound(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for a branch-and-bound based scan matcher */
    const auto& config = jsonSettings.get_child(configGroup);

    const std::string scanMatcherName = config.get<std::string>("Name");
    const int nodeHeightMax = config.get<int>("NodeHeightMax");
    const double rangeX = config.get<double>("SearchRangeX");
    const double rangeY = config.get<double>("SearchRangeY");
    const double rangeTheta = config.get<double>("SearchRangeTheta");

    /* Read settings for a new pixel-accurate score function evaluator */
    const std::string scoreType =
        config.get<std::string>("ScoreType");
    const std::string scoreConfigGroup =
        config.get<std::string>("ScoreConfigGroup");

    /* Make sure that the pixel-accurate score is used */
    Assert(scoreType == "PixelAccurate");

    /* Construct a new pixel-accurate score function evaluator */
    auto pScoreFunc = std::dynamic_pointer_cast<Mapping::ScorePixelAccurate>(
        CreateScoreFunction(jsonSettings, scoreType, scoreConfigGroup));

    /* Read settings for a new cost function evaluator */
    const std::string costType =
        config.get<std::string>("CostType");
    const std::string costConfigGroup =
        config.get<std::string>("CostConfigGroup");

    /* Construct a new cost function evaluator */
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct a new branch-and-bound based scan matcher */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherBranchBound>(
        scanMatcherName, pScoreFunc, pCostFunc,
        nodeHeightMax, rangeX, rangeY, rangeTheta);

    return pScanMatcher;
}

/* Create a new exhaustive grid search based scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherGridSearch(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for a new grid search based scan matcher */
    const auto& config = jsonSettings.get_child(configGroup);

    const std::string scanMatcherName = config.get<std::string>("Name");
    const double rangeX = config.get<double>("SearchRangeX");
    const double rangeY = config.get<double>("SearchRangeY");
    const double rangeTheta = config.get<double>("SearchRangeTheta");
    const double stepX = config.get<double>("SearchStepX");
    const double stepY = config.get<double>("SearchStepY");
    const double stepTheta = config.get<double>("SearchStepTheta");

    /* Read settings for a new pixel-accurate score function evaluator */
    const std::string scoreType =
        config.get<std::string>("ScoreType");
    const std::string scoreConfigGroup =
        config.get<std::string>("ScoreConfigGroup");

    /* Make sure that the pixel-accurate score is used */
    Assert(scoreType == "PixelAccurate");

    /* Construct a new pixel-accurate score function evaluator */
    auto pScoreFunc = std::dynamic_pointer_cast<Mapping::ScorePixelAccurate>(
        CreateScoreFunction(jsonSettings, scoreType, scoreConfigGroup));

    /* Read settings for a new cost function evaluator */
    const std::string costType =
        config.get<std::string>("CostType");
    const std::string costConfigGroup =
        config.get<std::string>("CostConfigGroup");

    /* Construct a new cost function evaluator */
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct a new grid search based scan matcher */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherGridSearch>(
        scanMatcherName, pScoreFunc, pCostFunc,
        rangeX, rangeY, rangeTheta, stepX, stepY, stepTheta);

    return pScanMatcher;
}

/* Create the greedy endpoint scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherHillClimbing(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for hill-climbing based scan matcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::string scanMatcherName = config.get<std::string>("Name");
    const double linearStep = config.get<double>("LinearStep");
    const double angularStep = config.get<double>("AngularStep");
    const int maxIterations = config.get<int>("MaxIterations");
    const int maxNumOfRefinements = config.get<int>("MaxNumOfRefinements");

    /* Construct cost function */
    const std::string costType =
        config.get<std::string>("CostType");
    const std::string costConfigGroup =
        config.get<std::string>("CostConfigGroup");
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct scan matcher */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherHillClimbing>(
        scanMatcherName, linearStep, angularStep,
        maxIterations, maxNumOfRefinements, pCostFunc);

    return pScanMatcher;
}

/* Create the linear solver scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherLinearSolver(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for linear solver scan matcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::string scanMatcherName = config.get<std::string>("Name");
    const int numOfIterationsMax = config.get<int>("NumOfIterationsMax");
    const double errorTolerance = config.get<double>("ConvergenceThreshold");
    const double initialLambda = config.get<double>("InitialLambda");

    /* Construct square error cost function */
    const std::string costType =
        config.get<std::string>("CostType");
    const std::string costConfigGroup =
        config.get<std::string>("CostConfigGroup");

    /* Check if the cost is calculated based on the squared error */
    Assert(costType == "SquareError");

    auto pCostFunc = std::dynamic_pointer_cast<Mapping::CostSquareError>(
        CreateCostSquareError(jsonSettings, costConfigGroup));

    /* Construct scan matcher */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherLinearSolver>(
        scanMatcherName, numOfIterationsMax, errorTolerance,
        initialLambda, pCostFunc);

    return pScanMatcher;
}

/* Create the real-time correlative scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherCorrelative(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for real-time correlative scan matcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::string scanMatcherName = config.get<std::string>("Name");
    const int lowResolution = config.get<int>("LowResolutionMapWinSize");
    const double rangeX = config.get<double>("SearchRangeX");
    const double rangeY = config.get<double>("SearchRangeY");
    const double rangeTheta = config.get<double>("SearchRangeTheta");

    /* Construct cost function */
    const std::string costType =
        config.get<std::string>("CostType");
    const std::string costConfigGroup =
        config.get<std::string>("CostConfigGroup");
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct the real-time correlative scan matcher */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherCorrelative>(
        scanMatcherName, pCostFunc, lowResolution,
        rangeX, rangeY, rangeTheta);

    return pScanMatcher;
}

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
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherCorrelativeFPGA(
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

/* Create the scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcher(
    const pt::ptree& jsonSettings,
    const std::string& scanMatcherType,
    const std::string& configGroup)
{
    if (scanMatcherType == "BranchBound")
        return CreateScanMatcherBranchBound(jsonSettings, configGroup);
    else if (scanMatcherType == "GridSearch")
        return CreateScanMatcherGridSearch(jsonSettings, configGroup);
    else if (scanMatcherType == "HillClimbing")
        return CreateScanMatcherHillClimbing(jsonSettings, configGroup);
    else if (scanMatcherType == "LinearSolver")
        return CreateScanMatcherLinearSolver(jsonSettings, configGroup);
    else if (scanMatcherType == "RealTimeCorrelative")
        return CreateScanMatcherCorrelative(jsonSettings, configGroup);

    return nullptr;
}

/* Create the nearest loop searcher */
std::unique_ptr<Mapping::LoopSearcherNearest> CreateLoopSearcherNearest(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for nearest loop searcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double travelDistThreshold =
        config.get<double>("TravelDistThreshold");
    const double nodeDistMax = config.get<double>("PoseGraphNodeDistMax");
    const int numOfCandidateNodes = config.get<int>("NumOfCandidateNodes");

    /* Construct loop searcher */
    auto pLoopSearcher = std::make_unique<Mapping::LoopSearcherNearest>(
        travelDistThreshold, nodeDistMax, numOfCandidateNodes);

    return pLoopSearcher;
}

/* Create the loop searcher */
std::unique_ptr<Mapping::LoopSearcher> CreateLoopSearcher(
    const pt::ptree& jsonSettings,
    const std::string& searcherType,
    const std::string& configGroup)
{
    if (searcherType == "Nearest")
        return CreateLoopSearcherNearest(jsonSettings, configGroup);

    return nullptr;
}

/* Create an empty loop detector object */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorEmpty(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for an empty loop detector */
    const auto& config = jsonSettings.get_child(configGroup);

    const std::string loopDetectorName =
        config.get<std::string>("LoopDetectorName");

    /* Construct an empty loop detector object */
    auto pLoopDetector = std::make_unique<Mapping::LoopDetectorEmpty>(
        loopDetectorName);

    return pLoopDetector;
}

/* Create a new exhaustive grid search based loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorGridSearch(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for a grid search loop detector */
    const auto& config = jsonSettings.get_child(configGroup);

    const std::string loopDetectorName =
        config.get<std::string>("LoopDetectorName");
    const double scoreThreshold =
        config.get<double>("ScoreThreshold");
    const double knownRateThreshold =
        config.get<double>("KnownRateThreshold");

    const std::string scanMatcherType =
        config.get<std::string>("ScanMatcherType");
    const std::string scanMatcherConfigGroup =
        config.get<std::string>("ScanMatcherConfigGroup");

    const std::string finalScanMatcherType =
        config.get<std::string>("FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("FinalScanMatcherConfigGroup");

    /* Make sure that the grid search based scan matcher is used */
    Assert(scanMatcherType == "GridSearch");

    /* Construct a new grid search based scan matcher */
    auto pScanMatcher = std::dynamic_pointer_cast<
        Mapping::ScanMatcherGridSearch>(
            CreateScanMatcher(jsonSettings, scanMatcherType,
                              scanMatcherConfigGroup));
    /* Construct a final scan matcher for refinements */
    auto pFinalScanMatcher = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup);

    /* Construct a new grid search based loop detector */
    auto pLoopDetector = std::make_unique<Mapping::LoopDetectorGridSearch>(
        loopDetectorName, pScanMatcher, pFinalScanMatcher,
        scoreThreshold, knownRateThreshold);

    return pLoopDetector;
}

/* Create a new real-time correlative loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorCorrelative(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for real-time correlative loop detection */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::string loopDetectorName =
        config.get<std::string>("LoopDetectorName");
    const double scoreThreshold =
        config.get<double>("ScoreThreshold");
    const double knownRateThreshold =
        config.get<double>("KnownRateThreshold");

    const std::string scanMatcherType =
        config.get<std::string>("ScanMatcherType");
    const std::string scanMatcherConfigGroup =
        config.get<std::string>("ScanMatcherConfigGroup");

    const std::string finalScanMatcherType =
        config.get<std::string>("FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("FinalScanMatcherConfigGroup");

    /* Make sure that the real-time correlative scan matcher is used */
    Assert(scanMatcherType == "RealTimeCorrelative");

    auto pScanMatcher = std::dynamic_pointer_cast<
        Mapping::ScanMatcherCorrelative>(
            CreateScanMatcher(jsonSettings, scanMatcherType,
                              scanMatcherConfigGroup));
    /* Construct a final scan matcher for refinements */
    auto pFinalScanMatcher = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup);

    /* Construct a real-time correlative loop detector */
    auto pLoopDetector = std::make_unique<Mapping::LoopDetectorCorrelative>(
        loopDetectorName, pScanMatcher, pFinalScanMatcher,
        scoreThreshold, knownRateThreshold);

    return pLoopDetector;
}

/* Create a new branch-and-bound loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorBranchBound(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for a branch-and-bound loop detector */
    const auto& config = jsonSettings.get_child(configGroup);

    const std::string loopDetectorName =
        config.get<std::string>("LoopDetectorName");
    const double scoreThreshold =
        config.get<double>("ScoreThreshold");
    const double knownRateThreshold =
        config.get<double>("KnownRateThreshold");

    const std::string scanMatcherType =
        config.get<std::string>("ScanMatcherType");
    const std::string scanMatcherConfigGroup =
        config.get<std::string>("ScanMatcherConfigGroup");

    const std::string finalScanMatcherType =
        config.get<std::string>("FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("FinalScanMatcherConfigGroup");

    /* Make sure that the branch-and-bound based scan matcher is used */
    Assert(scanMatcherType == "BranchBound");

    /* Construct a new branch-and-bound based scan matcher */
    auto pScanMatcher = std::dynamic_pointer_cast<
        Mapping::ScanMatcherBranchBound>(
            CreateScanMatcher(jsonSettings, scanMatcherType,
                              scanMatcherConfigGroup));
    /* Construct a final scan matcher for refinements */
    auto pFinalScanMatcher = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup);

    /* Construct a branch-and-bound loop detector object */
    auto pLoopDetector = std::make_unique<Mapping::LoopDetectorBranchBound>(
        loopDetectorName, pScanMatcher, pFinalScanMatcher,
        scoreThreshold, knownRateThreshold);

    return pLoopDetector;
}

/* Create a loop detector object */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetector(
    const pt::ptree& jsonSettings,
    const std::string& loopDetectorType,
    const std::string& configGroup)
{
    if (loopDetectorType == "GridSearch")
        return CreateLoopDetectorGridSearch(jsonSettings, configGroup);
    else if (loopDetectorType == "RealTimeCorrelative")
        return CreateLoopDetectorCorrelative(jsonSettings, configGroup);
    else if (loopDetectorType == "BranchBound")
        return CreateLoopDetectorBranchBound(jsonSettings, configGroup);
    else if (loopDetectorType == "Empty")
        return CreateLoopDetectorEmpty(jsonSettings, configGroup);

    return nullptr;
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

    const std::string finalScanMatcherType =
        config.get<std::string>("FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("FinalScanMatcherConfigGroup");

    /* Create a new real-time correlative scan matcher */
    auto pScanMatcher = std::dynamic_pointer_cast<
        Mapping::ScanMatcherCorrelativeFPGA>(
            CreateScanMatcherCorrelativeFPGA(
                jsonSettings, configGroup));
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

/* Create pose graph object */
std::shared_ptr<Mapping::PoseGraph> CreatePoseGraph(
    const pt::ptree& /* jsonSettings */)
{
    /* Construct pose graph object */
    auto pPoseGraph = std::make_shared<Mapping::PoseGraph>();
    return pPoseGraph;
}

/* Create squared loss function object (Gaussian) */
std::shared_ptr<Mapping::LossFunction> CreateLossSquared(
    const pt::ptree& /* jsonSettings */,
    const std::string& /* configGroup */)
{
    /* Construct squared loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossSquared>();
    return pLossFunction;
}

/* Create Huber loss function object */
std::shared_ptr<Mapping::LossFunction> CreateLossHuber(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for Huber loss function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);
    /* Preferred scale value is 1.345 * 1.345 */
    const double scale = config.get<double>("Scale");

    /* Construct Huber loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossHuber>(scale);
    return pLossFunction;
}

/* Create Cauchy loss function object */
std::shared_ptr<Mapping::LossFunction> CreateLossCauchy(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for Cauchy loss function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);
    /* Preferred scale value is 1e-2 */
    const double scale = config.get<double>("Scale");

    /* Construct Cauchy loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossCauchy>(scale);
    return pLossFunction;
}

/* Create Fair loss function object */
std::shared_ptr<Mapping::LossFunction> CreateLossFair(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for Fair loss function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);
    /* Preferred scale value is 1.3998 * 1.3998 */
    const double scale = config.get<double>("Scale");

    /* Construct Fair loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossFair>(scale);
    return pLossFunction;
}

/* Create Geman-McClure loss function object */
std::shared_ptr<Mapping::LossFunction> CreateLossGemanMcClure(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for Geman-McClure loss function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);
    /* Preferred scale value is 1.0 */
    const double scale = config.get<double>("Scale");

    /* Construct Geman-McClure loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossGemanMcClure>(scale);
    return pLossFunction;
}

/* Create Welsch loss function object */
std::shared_ptr<Mapping::LossFunction> CreateLossWelsch(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for Welsch loss function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);
    /* Preferred scale value is 2.9846 * 2.9846 */
    const double scale = config.get<double>("Scale");

    /* Construct Welsch loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossWelsch>(scale);
    return pLossFunction;
}

/* Create DCS (Dynamic Covariance Scaling) loss function object */
std::shared_ptr<Mapping::LossFunction> CreateLossDCS(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for DCS loss function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);
    /* Preferred scale value is 1.0 */
    const double scale = config.get<double>("Scale");

    /* Construct DCS loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossDCS>(scale);
    return pLossFunction;
}

/* Create loss function object */
std::shared_ptr<Mapping::LossFunction> CreateLossFunction(
    const pt::ptree& jsonSettings,
    const std::string& lossFunctionType,
    const std::string& configGroup)
{
    if (lossFunctionType == "Squared")
        return CreateLossSquared(jsonSettings, configGroup);
    else if (lossFunctionType == "Huber")
        return CreateLossHuber(jsonSettings, configGroup);
    else if (lossFunctionType == "Cauchy")
        return CreateLossCauchy(jsonSettings, configGroup);
    else if (lossFunctionType == "Fair")
        return CreateLossFair(jsonSettings, configGroup);
    else if (lossFunctionType == "GemanMcClure")
        return CreateLossGemanMcClure(jsonSettings, configGroup);
    else if (lossFunctionType == "Welsch")
        return CreateLossWelsch(jsonSettings, configGroup);
    else if (lossFunctionType == "DCS")
        return CreateLossDCS(jsonSettings, configGroup);

    return nullptr;
}

/* Create Levenberg-Marquardt method based pose graph optimizer object */
std::unique_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizerLM(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for Levenberg-Marquardt based pose graph optimizer */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    /* Convert linear solver type string to enum */
    using SolverType = Mapping::PoseGraphOptimizerLM::SolverType;
    const std::string solverTypeStr =
        config.get<std::string>("SolverType");
    const SolverType solverType =
        solverTypeStr == "SparseCholesky" ? SolverType::SparseCholesky :
        solverTypeStr == "ConjugateGradient" ? SolverType::ConjugateGradient :
        SolverType::SparseCholesky;

    const int numOfIterationsMax = config.get<int>("NumOfIterationsMax");
    const double errorTolerance = config.get<double>("ErrorTolerance");
    const double initialLambda = config.get<double>("InitialLambda");

    const std::string lossFuncType =
        config.get<std::string>("LossFunctionType");
    const std::string lossFuncConfigGroup =
        config.get<std::string>("LossFunctionConfigGroup");
    auto pLossFunction = CreateLossFunction(
        jsonSettings, lossFuncType, lossFuncConfigGroup);

    /* Construct pose graph optimizer object */
    auto pOptimizer = std::make_unique<Mapping::PoseGraphOptimizerLM>(
        solverType, numOfIterationsMax, errorTolerance, initialLambda,
        pLossFunction);

    return pOptimizer;
}

/* Create g2o-based pose graph optimizer */
std::unique_ptr<Mapping::PoseGraphOptimizerG2O> CreatePoseGraphOptimizerG2O(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for g2o-based pose graph optimizer */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const int maxNumOfIterations =
        config.get<int>("MaxNumOfIterations");
    const double convergenceThreshold =
        config.get<double>("ConvergenceThreshold");

    /* Create g2o-based pose graph optimizer */
    auto pOptimizer = std::make_unique<Mapping::PoseGraphOptimizerG2O>(
        maxNumOfIterations, convergenceThreshold);

    return pOptimizer;
}

/* Create pose graph optimizer object */
std::unique_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizer(
    const pt::ptree& jsonSettings,
    const std::string& optimizerType,
    const std::string& configGroup)
{
    if (optimizerType == "LM")
        return CreatePoseGraphOptimizerLM(jsonSettings, configGroup);
    else if (optimizerType == "G2O")
        return CreatePoseGraphOptimizerG2O(jsonSettings, configGroup);

    return nullptr;
}

/* Create a scan outlier filter */
std::shared_ptr<Mapping::ScanOutlierFilter> CreateScanOutlierFilter(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for a scan outlier filter */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double validRangeMin = config.get<double>("ValidRangeMin");
    const double validRangeMax = config.get<double>("ValidRangeMax");

    /* Create a scan outlier filter */
    auto pScanOutlierFilter = std::make_shared<Mapping::ScanOutlierFilter>(
        validRangeMin, validRangeMax);

    return pScanOutlierFilter;
}

/* Create scan accumulator object */
std::shared_ptr<Mapping::ScanAccumulator> CreateScanAccumulator(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for scan accumulator */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::size_t numOfAccumulatedScans =
        config.get<std::size_t>("NumOfAccumulatedScans");

    /* Construct scan accumulator object */
    auto pScanAccumulator = std::make_shared<Mapping::ScanAccumulator>(
        numOfAccumulatedScans);

    return pScanAccumulator;
}

/* Create scan interpolator object */
std::shared_ptr<Mapping::ScanInterpolator> CreateScanInterpolator(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for scan interpolator */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double distScans = config.get<double>("DistScans");
    const double distThresholdEmpty = config.get<double>("DistThresholdEmpty");

    /* Construct scan interpolator object */
    auto pScanInterpolator = std::make_shared<Mapping::ScanInterpolator>(
        distScans, distThresholdEmpty);

    return pScanInterpolator;
}

/* Create grid map builder object */
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

/* Create SLAM frontend */
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
            jsonSettings, localScanMatcherType, localScanMatcherConfigGroup);
    }

    /* Create the final scan matcher for local SLAM */
    const std::string finalScanMatcherType =
        config.get<std::string>("LocalSlam.FinalScanMatcherType");
    const std::string finalScanMatcherConfigGroup =
        config.get<std::string>("LocalSlam.FinalScanMatcherConfigGroup");

    auto pFinalScanMatcher = CreateScanMatcher(
        jsonSettings, finalScanMatcherType, finalScanMatcherConfigGroup);

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

/* Create SLAM backend */
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

    std::unique_ptr<Mapping::LoopDetector> pLoopDetector = nullptr;

    if (useHardwareLoopDetector) {
        /* Create the FPGA-based loop detector */
        pLoopDetector = CreateLoopDetectorCorrelativeFPGA(
            jsonSettings, "Hardware.LoopDetector");
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

/* Create LiDAR Graph-Based SLAM object */
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
    auto pPoseGraph = CreatePoseGraph(jsonSettings);

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

/* Load Carmen log data */
void LoadCarmenLog(const fs::path& logFilePath,
                   std::vector<Sensor::SensorDataPtr>& logData)
{
    /* Open the carmen log file */
    std::ifstream logFile { logFilePath };

    if (!logFile) {
        std::cerr << "Failed to open log file: " << logFilePath << std::endl;
        return;
    }

    /* Load the carmen log file */
    IO::Carmen::CarmenLogReader logReader;
    logReader.Load(logFile, logData);
    logFile.close();
}

/* LauncherSettings struct stores configurations for SLAM launcher */
struct LauncherSettings
{
    bool        mGuiEnabled;
    int         mDrawFrameInterval;
    bool        mWaitForKey;
};

/* Load the bitstream file to enable the hardware acceleration */
bool LoadBitstream(const pt::ptree& jsonSettings)
{
    /* Read settings for the hardware acceleration */
    const bool enableHardwareAcceleration =
        jsonSettings.get<bool>("Hardware.EnableHardwareAcceleration");
    const std::string bitstreamFileName =
        jsonSettings.get<std::string>("Hardware.BitstreamFileName");

    if (!enableHardwareAcceleration)
        return true;

    /* Load the shared object library for the CMA memory management */
    auto* const pCmaManager = Hardware::CMAMemoryManager::Instance();

    if (!pCmaManager->Load())
        return false;

    /* Load the specified bitstream file */
    Hardware::BitstreamLoader bitstreamLoader;

    if (!bitstreamLoader.Load(bitstreamFileName))
        return false;

    return true;
}

/* Load settings from JSON format configuration file */
bool LoadSettings(const fs::path& settingsFilePath,
                  Mapping::LidarGraphSlamPtr& pLidarGraphSlam,
                  LauncherSettings& launcherSettings)
{
    /* Load settings from JSON configuration file */
    pt::ptree jsonSettings;
    pt::read_json(settingsFilePath, jsonSettings);

    /* Read settings for gnuplot GUI */
    launcherSettings.mGuiEnabled =
        jsonSettings.get<bool>("Launcher.GuiEnabled");
    launcherSettings.mDrawFrameInterval =
        jsonSettings.get<int>("Launcher.DrawFrameInterval");

    /* Read settings for SLAM launcher */
    launcherSettings.mWaitForKey =
        jsonSettings.get<bool>("Launcher.WaitForKey");

    /* Load the bitstream file to enable the hardware acceleration
     * before setting up the scan matcher and the loop detector */
    if (!LoadBitstream(jsonSettings))
        return false;

    /* Construct LiDAR Graph-Based SLAM */
    pLidarGraphSlam = CreateLidarGraphSlam(jsonSettings);

    return true;
}

/* Draw pose graph on Gnuplot window */
void DrawPoseGraph(const Mapping::LidarGraphSlamPtr& pLidarGraphSlam,
                   const std::unique_ptr<IO::GnuplotHelper>& pGnuplotHelper)
{
    /* Get the pose graph information */
    Mapping::IdMap<Mapping::LocalMapId, Mapping::LocalMapNode> localMapNodes;
    Mapping::IdMap<Mapping::NodeId, Mapping::ScanNodeData> scanNodes;
    std::vector<Mapping::EdgeData> poseGraphEdges;
    pLidarGraphSlam->GetPoseGraph(localMapNodes, scanNodes, poseGraphEdges);

    /* Draw the current pose graph if necessary */
    pGnuplotHelper->DrawPoseGraph(localMapNodes, scanNodes, poseGraphEdges);
}

/* Save the metrics */
void SaveMetrics(const std::string& fileName)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();
    /* Convert all metrics to the property tree */
    const pt::ptree metricsTree = pMetricManager->ToPropertyTree();

    /* Write metrics to the JSON file */
    const std::string metricsFileName = fileName + ".metric.json";
    pt::write_json(metricsFileName, metricsTree);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << ' '
                  << "<Carmen log file name> "
                  << "<JSON Settings file name> "
                  << "[Output name]" << std::endl;
        return EXIT_FAILURE;
    }

    fs::path logFilePath { argv[1] };
    fs::path settingsFilePath { argv[2] };

    /* Determine the output file name */
    const bool hasValidFileName = logFilePath.has_stem() &&
                                  logFilePath.stem() != "." &&
                                  logFilePath.stem() != "..";
    fs::path outputFilePath { (argc == 4 || !hasValidFileName) ? argv[3] :
                              logFilePath.stem() };

    /* Load Carmen log file */
    std::vector<Sensor::SensorDataPtr> logData;
    LoadCarmenLog(logFilePath, logData);

    if (logData.empty())
        return EXIT_FAILURE;

    /* Load settings from JSON configuration file */
    Mapping::LidarGraphSlamPtr pLidarGraphSlam;
    LauncherSettings launcherSettings;

    if (!LoadSettings(settingsFilePath, pLidarGraphSlam, launcherSettings))
        return EXIT_FAILURE;

    /* Start the SLAM backend */
    pLidarGraphSlam->StartBackend();

    /* Setup gnuplot helper */
    auto pGnuplotHelper = launcherSettings.mGuiEnabled ?
        std::make_unique<IO::GnuplotHelper>() : nullptr;

    for (auto& sensorData : logData) {
        auto scanData = std::dynamic_pointer_cast<
            Sensor::ScanData<double>>(std::move(sensorData));

        if (scanData == nullptr)
            continue;

        /* Process the latest scan data */
        const bool mapUpdated = pLidarGraphSlam->ProcessScan(
            scanData, scanData->OdomPose());

        if (!launcherSettings.mGuiEnabled || !mapUpdated)
            continue;
        if (pLidarGraphSlam->ProcessCount() %
            launcherSettings.mDrawFrameInterval != 0)
            continue;

        /* Draw the current pose graph if necessary */
        DrawPoseGraph(pLidarGraphSlam, pGnuplotHelper);
    }

    /* Stop the SLAM backend */
    pLidarGraphSlam->StopBackend();

    /* Draw the final pose graph if necessary */
    if (launcherSettings.mGuiEnabled)
        DrawPoseGraph(pLidarGraphSlam, pGnuplotHelper);

    IO::MapSaver* const pMapSaver = IO::MapSaver::Instance();

    /* Retrieve a latest map that contains latest scans */
    RobotPose2D<double> latestMapPose;
    Mapping::GridMap latestMap;
    Mapping::NodeId latestMapNodeIdMin { Mapping::NodeId::Invalid };
    Mapping::NodeId latestMapNodeIdMax { Mapping::NodeId::Invalid };
    pLidarGraphSlam->GetLatestMap(latestMapPose, latestMap,
                                  latestMapNodeIdMin, latestMapNodeIdMax);

    /* Retrieve all pose graph nodes and edges */
    Mapping::IdMap<Mapping::LocalMapId,
        Mapping::LocalMapNode> localMapNodes;
    Mapping::IdMap<Mapping::NodeId,
        Mapping::ScanNode> scanNodes;
    std::vector<Mapping::PoseGraphEdge> poseGraphEdges;
    pLidarGraphSlam->GetPoseGraph(localMapNodes, scanNodes, poseGraphEdges);
    const Mapping::NodeId scanNodeIdMin = scanNodes.IdMin();
    const Mapping::NodeId scanNodeIdMax = scanNodes.IdMax();

    /* Build a global map that contains all local grid maps */
    RobotPose2D<double> globalMapPose;
    Mapping::GridMap globalMap;
    pLidarGraphSlam->GetGlobalMap(globalMapPose, globalMap,
                                  scanNodeIdMin, scanNodeIdMax);

    /* Save the global map, the pose graph, and the latest map */
    pMapSaver->SaveMap(
        globalMapPose, globalMap, &scanNodes,
        &scanNodeIdMin, &scanNodeIdMax, true, outputFilePath);
    pMapSaver->SavePoseGraph(
        localMapNodes, scanNodes, poseGraphEdges, outputFilePath);
    pMapSaver->SaveLatestMapAndScan(
        latestMapPose, latestMap, &scanNodes,
        &latestMapNodeIdMin, &latestMapNodeIdMax,
        nullptr, nullptr, true, outputFilePath);

    /* Save the metrics */
    SaveMetrics(outputFilePath);

    if (launcherSettings.mWaitForKey)
        std::getchar();

    return EXIT_SUCCESS;
}
