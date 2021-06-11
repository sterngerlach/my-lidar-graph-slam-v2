
/* grid_map_types.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_GRID_MAP_TYPES_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_GRID_MAP_TYPES_HPP

#include "my_lidar_graph_slam/grid_map_new/grid_binary_bayes.hpp"
#include "my_lidar_graph_slam/grid_map_new/grid_constant.hpp"
#include "my_lidar_graph_slam/grid_map_new/grid_map.hpp"

#include <type_traits>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Make sure that grid maps and precomputed grid maps store and represent
 * occupancy probability values using the same data type */
static_assert(std::is_same<GridMapNew::GridBinaryBayes::ProbabilityType,
                           GridMapNew::GridConstant::ProbabilityType>::value,
              "GridBinaryBayes and GridConstant should represent "
              "occupancy probability values using the same data type");
static_assert(std::is_same<GridMapNew::GridBinaryBayes::ValueType,
                           GridMapNew::GridConstant::ValueType>::value,
              "GridBinaryBayes and GridConstant should store "
              "occupancy probability values using the same data type");

/* Type definitions */
using GridMap = GridMapNew::GridMap<GridMapNew::GridBinaryBayes>;
using ConstMap = GridMapNew::GridMap<GridMapNew::GridConstant>;
using GridMapInterface = GridMapNew::GridMapInterface<
    GridMapNew::GridBinaryBayes::ProbabilityType,
    GridMapNew::GridBinaryBayes::ValueType>;

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_GRID_MAP_TYPES_HPP */
