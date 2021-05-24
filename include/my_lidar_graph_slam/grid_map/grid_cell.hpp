
/* grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_CELL_HPP

namespace MyLidarGraphSlam {

template <typename T, typename U, typename V>
class GridCell
{
public:
    /* Type definition */
    using ValueType = T;
    using StorageType = U;
    using ObservationType = V;

    /* Default constructor */
    GridCell() = default;
    /* Destructor */
    virtual ~GridCell() = default;

    /* Copy constructor */
    GridCell(const GridCell&) = default;
    /* Move constructor */
    GridCell(GridCell&&) noexcept = default;
    /* Copy assignment operator */
    GridCell& operator=(const GridCell&) = default;
    /* Move assignment operator */
    GridCell& operator=(GridCell&&) noexcept = default;

    /* Cast operator */
    virtual explicit operator T() const = 0;
    /* Get the value of the grid cell */
    virtual T Value() const = 0;
    /* Get the internal value of the grid cell */
    virtual U RawValue() const = 0;

    /* Reset the grid cell state */
    virtual void Reset() = 0;

    /* Set the value of the grid cell */
    virtual void SetValue(const T newValue) = 0;
    /* Set the internal value of the grid cell */
    virtual void SetRawValue(const U newRawValue) = 0;

    /* Update the value of the grid cell */
    virtual void Update(const V currentObservation) = 0;

    /* Unknown occupancy probability value */
    static constexpr T Unknown = static_cast<T>(0);
    /* Unknown occupancy probability value for internal representation */
    static constexpr U UnknownRaw = static_cast<U>(0);
};

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_CELL_HPP */
