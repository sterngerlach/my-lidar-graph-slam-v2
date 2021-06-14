
/* cost_function_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_COST_FUNCTION_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_COST_FUNCTION_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_greedy_endpoint.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_square_error.hpp"

namespace MyLidarGraphSlam {

/* Create the greedy endpoint cost function */
std::shared_ptr<Mapping::CostFunction> CreateCostGreedyEndpoint(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create the square error cost function */
std::shared_ptr<Mapping::CostFunction> CreateCostSquareError(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create the cost function */
std::shared_ptr<Mapping::CostFunction> CreateCostFunction(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& costType,
    const std::string& configGroup);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_COST_FUNCTION_FACTORY_HPP */
