
/* scan_matcher_factory.cpp */

#include "my_lidar_graph_slam/scan_matcher_factory.hpp"

#include "my_lidar_graph_slam/cost_function_factory.hpp"
#include "my_lidar_graph_slam/score_function_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

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

/* Create a greedy endpoint scan matcher */
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

/* Create a linear solver based scan matcher */
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

/* Create a real-time correlative scan matcher */
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

/* Create a scan matcher */
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

} /* namespace MyLidarGraphSlam */
