
/* loop_detector_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_LOOP_DETECTOR_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_LOOP_DETECTOR_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_branch_bound.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_empty.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_grid_search.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_correlative.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher_nearest.hpp"

namespace MyLidarGraphSlam {

/* Create a nearest loop searcher */
std::unique_ptr<Mapping::LoopSearcherNearest> CreateLoopSearcherNearest(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a loop searcher */
std::unique_ptr<Mapping::LoopSearcher> CreateLoopSearcher(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& searcherType,
    const std::string& configGroup);

/* Create an empty loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorEmpty(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a exhaustive grid search based loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorGridSearch(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a real-time correlative loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorCorrelative(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a branch-and-bound loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorBranchBound(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a loop detector */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetector(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& loopDetectorType,
    const std::string& configGroup);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_LOOP_DETECTOR_FACTORY_HPP */
