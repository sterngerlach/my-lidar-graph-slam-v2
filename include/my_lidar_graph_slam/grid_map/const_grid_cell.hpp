
/* const_grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_CONST_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_CONST_GRID_CELL_HPP

#include "my_lidar_graph_slam/grid_map/grid_cell.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>

namespace MyLidarGraphSlam {

class ConstGridCell final : public GridCell<double, std::uint16_t, double>
{
public:
    /* Base type */
    using BaseType = GridCell<double, std::uint16_t, double>;

    /* Default constructor */
    ConstGridCell() : BaseType(),
                      mValue(UnknownRaw) { }
    /* Destructor */
    ~ConstGridCell() = default;

    /* Default copy constructor */
    ConstGridCell(const ConstGridCell&) = default;
    /* Default copy assignment operator */
    ConstGridCell& operator=(const ConstGridCell&) = default;
    /* Default move constructor */
    ConstGridCell(ConstGridCell&&) noexcept = default;
    /* Default move assignment operator */
    ConstGridCell& operator=(ConstGridCell&&) noexcept = default;

    /* Cast operator */
    explicit operator double() const override
    { return RawToValue(this->mValue); }
    /* Get the occupancy probability value of the grid cell */
    double Value() const override
    { return RawToValue(this->mValue); }
    /* Get the internal representation of the grid cell */
    std::uint16_t RawValue() const override { return this->mValue; }

    /* Reset the grid cell state */
    void Reset() override { this->mValue = UnknownRaw; }

    /* Set the value of the grid cell */
    void SetValue(const double newValue) override
    { this->mValue = ValueToRaw(newValue); }
    /* Set the internal value of the grid cell */
    void SetRawValue(const std::uint16_t newRawValue) override
    { this->mValue = newRawValue; }

    /* Update the value of the grid cell */
    void Update(const double newValue) override
    { this->mValue = ValueToRaw(newValue); }

    /* Smallest possible occupancy probability value */
    static constexpr std::uint16_t ValueMin = 1;
    /* Largest possible occupancy probability value */
    static constexpr std::uint16_t ValueMax =
        std::numeric_limits<std::uint16_t>::max();
    /* Occupancy probability value range */
    static constexpr std::uint16_t ValueRange = ValueMax - ValueMin;

    /* Convert the internal representation to the probability value */
    static double RawToValue(const std::uint16_t discretizedValue);
    /* Convert the probability value to the internal representation */
    static std::uint16_t ValueToRaw(const double probValue);

private:
    /* Occupancy probability value */
    std::uint16_t mValue;
};

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_CONST_GRID_CELL_HPP */
