
/* scan_filter_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_SCAN_FILTER_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_SCAN_FILTER_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/scan_accumulator.hpp"
#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"
#include "my_lidar_graph_slam/mapping/scan_outlier_filter.hpp"

namespace MyLidarGraphSlam {

/* Create a scan outlier filter */
std::shared_ptr<Mapping::ScanOutlierFilter> CreateScanOutlierFilter(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a scan accumulator */
std::shared_ptr<Mapping::ScanAccumulator> CreateScanAccumulator(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create a scan interpolator */
std::shared_ptr<Mapping::ScanInterpolator> CreateScanInterpolator(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_SCAN_FILTER_FACTORY_HPP */
