
/* pose_graph_optimizer_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_g2o.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_lm.hpp"

namespace MyLidarGraphSlam {

/* Create a Levenberg-Marquardt method based pose graph optimizer */
std::unique_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizerLM(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a g2o-based pose graph optimizer */
std::unique_ptr<Mapping::PoseGraphOptimizerG2O> CreatePoseGraphOptimizerG2O(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a pose graph optimizer */
std::unique_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizer(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& optimizerType,
    const std::string& configGroup);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_FACTORY_HPP */
