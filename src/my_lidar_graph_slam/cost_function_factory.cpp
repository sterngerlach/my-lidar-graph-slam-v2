
/* cost_function_factory.cpp */

#include "my_lidar_graph_slam/cost_function_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Create the greedy endpoint cost function */
std::shared_ptr<Mapping::CostFunction> CreateCostGreedyEndpoint(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for the greedy endpoint cost function */
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

/* Create the square error cost function */
std::shared_ptr<Mapping::CostFunction> CreateCostSquareError(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for the square error cost function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double covarianceScale = config.get<double>("CovarianceScale");
    
    /* Create cost function object */
    auto pCostFunc = std::make_shared<Mapping::CostSquareError>(
        covarianceScale);
    
    return pCostFunc;
}

/* Create the cost function */
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

} /* namespace MyLidarGraphSlam */
