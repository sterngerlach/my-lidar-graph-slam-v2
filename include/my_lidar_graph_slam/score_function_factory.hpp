
/* score_function_factory.hpp */

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"

namespace MyLidarGraphSlam {

/* Create the pixel-accurate score function */
std::shared_ptr<Mapping::ScoreFunction> CreateScorePixelAccurate(
    const boost::property_tree::ptree& /* jsonSettings */,
    const std::string& /* configGroup */);

/* Create the score function */
std::shared_ptr<Mapping::ScoreFunction> CreateScoreFunction(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& scoreType,
    const std::string& configGroup);

} /* namespace MyLidarGraphSlam */
