
/* loss_function_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_LOSS_FUNCTION_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_LOSS_FUNCTION_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/robust_loss_function.hpp"

namespace MyLidarGraphSlam {

/* Create a squared loss function (Gaussian) */
std::shared_ptr<Mapping::LossFunction> CreateLossSquared(
    const boost::property_tree::ptree& /* jsonSettings */,
    const std::string& /* configGroup */);

/* Create a Huber loss function */
std::shared_ptr<Mapping::LossFunction> CreateLossHuber(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a Cauchy loss function */
std::shared_ptr<Mapping::LossFunction> CreateLossCauchy(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a Fair loss function */
std::shared_ptr<Mapping::LossFunction> CreateLossFair(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a Geman-McClure loss function */
std::shared_ptr<Mapping::LossFunction> CreateLossGemanMcClure(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a Welsch loss function */
std::shared_ptr<Mapping::LossFunction> CreateLossWelsch(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a DCS (Dynamic Covariance Scaling) loss function */
std::shared_ptr<Mapping::LossFunction> CreateLossDCS(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a loss function */
std::shared_ptr<Mapping::LossFunction> CreateLossFunction(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& lossFunctionType,
    const std::string& configGroup);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_LOSS_FUNCTION_FACTORY_HPP */
