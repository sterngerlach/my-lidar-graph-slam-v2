
/* slam_module_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_SLAM_MODULE_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_SLAM_MODULE_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_frontend.hpp"

namespace MyLidarGraphSlam {

/* Create a grid map builder */
std::shared_ptr<Mapping::GridMapBuilder> CreateGridMapBuilder(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a SLAM frontend */
std::shared_ptr<Mapping::LidarGraphSlamFrontend> CreateSlamFrontend(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a SLAM backend */
std::shared_ptr<Mapping::LidarGraphSlamBackend> CreateSlamBackend(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a LiDAR Graph-Based SLAM module */
std::shared_ptr<Mapping::LidarGraphSlam> CreateLidarGraphSlam(
    const boost::property_tree::ptree& jsonSettings);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_SLAM_MODULE_FACTORY_HPP */
