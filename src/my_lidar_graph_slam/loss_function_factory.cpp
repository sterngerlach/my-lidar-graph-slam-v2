
/* loss_function_factory.cpp */

#include "my_lidar_graph_slam/loss_function_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Create a squared loss function (Gaussian) */
std::shared_ptr<Mapping::LossFunction> CreateLossSquared(
    const pt::ptree& /* jsonSettings */,
    const std::string& /* configGroup */)
{
    /* Construct squared loss function object */
    auto pLossFunction = std::make_shared<Mapping::LossSquared>();
    return pLossFunction;
}

/* Create a Huber loss function */
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

/* Create a Cauchy loss function */
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

/* Create a Fair loss function */
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

/* Create a Geman-McClure loss function */
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

/* Create a Welsch loss function */
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

/* Create a DCS (Dynamic Covariance Scaling) loss function */
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

/* Create a loss function */
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

} /* namespace MyLidarGraphSlam */
