
/* scan_matcher_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_SCAN_MATCHER_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_SCAN_MATCHER_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_branch_bound.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_correlative.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_grid_search.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_hill_climbing.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_linear_solver.hpp"

namespace MyLidarGraphSlam {

/* Create a new branch-and-bound based scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherBranchBound(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup,
    const std::string& namePostfix);

/* Create a new exhaustive grid search based scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherGridSearch(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup,
    const std::string& namePostfix);

/* Create a greedy endpoint scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherHillClimbing(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup,
    const std::string& namePostfix);

/* Create a linear solver based scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherLinearSolver(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup,
    const std::string& namePostfix);

/* Create a real-time correlative scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherCorrelative(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup,
    const std::string& namePostfix);

/* Create a scan matcher */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcher(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& scanMatcherType,
    const std::string& configGroup,
    const std::string& namePostfix);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_SCAN_MATCHER_FACTORY_HPP */
