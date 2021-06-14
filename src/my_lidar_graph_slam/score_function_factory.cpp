
/* score_function_factory.cpp */

#include "my_lidar_graph_slam/score_function_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Create the pixel-accurate score function */
std::shared_ptr<Mapping::ScoreFunction> CreateScorePixelAccurate(
    const pt::ptree& /* jsonSettings */,
    const std::string& /* configGroup */)
{
    /* Create score function object */
    auto pScoreFunc = std::make_shared<Mapping::ScorePixelAccurate>();
    return pScoreFunc;
}

/* Create the score function */
std::shared_ptr<Mapping::ScoreFunction> CreateScoreFunction(
    const pt::ptree& jsonSettings,
    const std::string& scoreType,
    const std::string& configGroup)
{
    if (scoreType == "PixelAccurate")
        return CreateScorePixelAccurate(jsonSettings, configGroup);
    
    return nullptr;
}

} /* namespace MyLidarGraphSlam */
