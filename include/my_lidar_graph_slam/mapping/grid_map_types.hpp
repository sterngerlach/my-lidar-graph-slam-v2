
/* grid_map_types.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_GRID_MAP_TYPES_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_GRID_MAP_TYPES_HPP

#include "my_lidar_graph_slam/grid_map/discrete_grid_cell.hpp"
#include "my_lidar_graph_slam/grid_map/const_grid_cell.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map.hpp"

#include <type_traits>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Make sure that grid maps and precomputed grid maps store and represent
 * occupancy probability values using the same data type */
static_assert(std::is_same<DiscreteGridCell::ValueType,
                           ConstGridCell::ValueType>::value,
              "DiscreteGridCell and ConstGridCell should represent "
              "occupancy probability values using the same data type");
static_assert(std::is_same<DiscreteGridCell::StorageType,
                           ConstGridCell::StorageType>::value,
              "DiscreteGridCell and ConstGridCell should store "
              "occupancy probability values using the same data type");

/* Type definitions */
using GridMapType = GridMap<DiscreteGridCell>;
using ConstMapType = GridMap<ConstGridCell>;
using GridMapInterfaceType = GridMapBase<DiscreteGridCell::ValueType,
                                         DiscreteGridCell::StorageType>;

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_GRID_MAP_TYPES_HPP */
