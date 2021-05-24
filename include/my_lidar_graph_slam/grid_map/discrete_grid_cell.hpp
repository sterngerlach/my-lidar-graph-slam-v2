
/* discrete_grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_DISCRETE_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_DISCRETE_GRID_CELL_HPP

#include "my_lidar_graph_slam/grid_map/grid_cell.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>

namespace MyLidarGraphSlam {

class DiscreteGridCell final : public GridCell<double, std::uint16_t, double>
{
public:
    /* Base type */
    using BaseType = GridCell<double, std::uint16_t, double>;

    /* Default constructor */
    DiscreteGridCell() : BaseType(),
                         mValue(UnknownRaw) { }
    /* Destructor */
    ~DiscreteGridCell() = default;

    /* Copy constructor */
    DiscreteGridCell(const DiscreteGridCell&) = default;
    /* Copy assignment operator */
    DiscreteGridCell& operator=(const DiscreteGridCell&) noexcept = default;
    /* Move constructor */
    DiscreteGridCell(DiscreteGridCell&&) = default;
    /* Move assignment operator */
    DiscreteGridCell& operator=(DiscreteGridCell&&) noexcept = default;

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
    void Update(const double probValue) override;

    /* Smallest possible occupancy probability value */
    static constexpr std::uint16_t ValueMin = 1;
    /* Largest possible occupancy probability value */
    static constexpr std::uint16_t ValueMax =
        std::numeric_limits<std::uint16_t>::max();
    /* Occupancy probability value range */
    static constexpr std::uint16_t ValueRange = ValueMax - ValueMin;

    /* Smallest possible occupancy probability value */
    static constexpr double ProbabilityMin = 1e-3;
    /* Largest possible occupancy probability value */
    static constexpr double ProbabilityMax = 1.0 - 1e-3;

    /* Convert the internal representation to the probability value */
    static double RawToValue(const std::uint16_t discretizedValue);
    /* Convert the probability value to the internal representation */
    static std::uint16_t ValueToRaw(const double probValue);

    /* Compute an odds from the occupancy probability value */
    static double ValueToOdds(const double probValue);
    /* Compute an occupancy probability value from the odds */
    static double OddsToValue(const double oddsValue);

private:
    /* Discretized occupancy probability value */
    std::uint16_t mValue;
};

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_DISCRETE_GRID_CELL_HPP */
