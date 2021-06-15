
/* fpga_module_factory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_FPGA_MODULE_FACTORY_HPP
#define MY_LIDAR_GRAPH_SLAM_FPGA_MODULE_FACTORY_HPP

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_correlative_fpga.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_fpga_parallel.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_correlative_fpga.hpp"

namespace MyLidarGraphSlam {

/* Load the register offsets for the real-time correlative-based
 * scan matcher IP core */
void LoadScanMatcherHardwareRegisterOffsets(
    const boost::property_tree::ptree& jsonSettings,
    Mapping::ScanMatcherHardwareConfig& scanMatcherConfig);

/* Create the real-time correlative-based scan matcher IP core interface */
std::shared_ptr<Mapping::ScanMatcherCorrelativeFPGA>
    CreateScanMatcherCorrelativeFPGA(
        const boost::property_tree::ptree& jsonSettings,
        const std::string& configGroup);

/* Create the real-time correlative-based loop detector IP core interface */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorCorrelativeFPGA(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

/* Create the real-time correlative-based loop detector which uses two IP cores
 * at the same time to improve the loop detection performance */
std::unique_ptr<Mapping::LoopDetector> CreateLoopDetectorFPGAParallel(
    const boost::property_tree::ptree& jsonSettings,
    const std::string& configGroup);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_FPGA_MODULE_FACTORY_HPP */
