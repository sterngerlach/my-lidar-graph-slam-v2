
/* loop_detector_factory.cpp */

#include "my_lidar_graph_slam/loop_detector_factory.hpp"

#include "my_lidar_graph_slam/scan_matcher_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Create a nearest loop searcher */
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

/* Create a loop searcher */
std::unique_ptr<Mapping::LoopSearcher> CreateLoopSearcher(
    const pt::ptree& jsonSettings,
    const std::string& searcherType,
    const std::string& configGroup)
{
    if (searcherType == "Nearest")
        return CreateLoopSearcherNearest(jsonSettings, configGroup);

    return nullptr;
}

/* Create an empty loop detector */
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

/* Create a exhaustive grid search based loop detector */
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

/* Create a real-time correlative loop detector */
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

/* Create a branch-and-bound loop detector */
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

/* Create a loop detector */
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

} /* namespace MyLidarGraphSlam */
