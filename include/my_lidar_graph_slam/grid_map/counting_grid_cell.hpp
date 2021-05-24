
/* counting_grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_COUNTING_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_COUNTING_GRID_CELL_HPP

#include "my_lidar_graph_slam/grid_map/grid_cell.hpp"
#include "my_lidar_graph_slam/util.hpp"

#include <cstdint>
#include <utility>

namespace MyLidarGraphSlam {

template <typename T>
class CountingGridCell final : public GridCell<T, T, bool>
{
public:
    /* Base type */
    using BaseType = GridCell<T, T, bool>;

    /* Default constructor */
    CountingGridCell() : BaseType(),
                         mValue(Unknown),
                         mHit(0),
                         mMiss(0) { }
    /* Destructor */
    ~CountingGridCell() = default;

    /* Copy constructor */
    CountingGridCell(const CountingGridCell&) = default;
    /* Copy assignment operator */
    CountingGridCell& operator=(const CountingGridCell&) = default;
    /* Move constructor */
    CountingGridCell(CountingGridCell&&) noexcept = default;
    /* Move assignment operator */
    CountingGridCell& operator=(CountingGridCell&&) noexcept = default;

    /* Cast operator */
    explicit operator T() const override { return this->mValue; }
    /* Get the value of the grid cell */
    T Value() const override { return this->mValue; }
    /* Get the internal representation of the grid cell */
    T RawValue() const override { return this->mValue; }

    /* Reset the grid cell state */
    void Reset() override;

    /* Set the value of the grid cell (Not implemented) */
    void SetValue(const T newValue) override
    { XAssert(false, "SetValue() is not implemented"); }
    /* Set the internal value of the grid cell (Not implemented) */
    void SetRawValue(const T newRawValue) override
    { XAssert(false, "SetRawValue() is not implemented"); }

    /* Update the value of the grid cell */
    void Update(const bool hitOrMiss) override;

    /* Unknown occupancy probability value */
    using BaseType::Unknown;
    using BaseType::UnknownRaw;

    /* Smallest possible occupancy probability value,
     * which is slightly larger than the unknown value */
    static constexpr T Epsilon = static_cast<T>(1e-3);

private:
    T             mValue;
    std::uint32_t mHit;
    std::uint32_t mMiss;
};

/* Reset the grid cell state */
template <typename T>
void CountingGridCell<T>::Reset()
{
    this->mValue = Unknown;
    this->mHit = 0;
    this->mMiss = 0;
}

/* Update the value of the grid cell */
template <typename T>
void CountingGridCell<T>::Update(const bool hitOrMiss)
{
    /* Update the counter */
    if (hitOrMiss)
        ++this->mHit;
    else
        ++this->mMiss;
    
    /* Update the occupancy probability value */
    this->mValue = static_cast<T>(this->mHit) /
                   static_cast<T>(this->mHit + this->mMiss);
    
    /* If the number of the hits is zero, then add the small value
     * to distinguish from the unknown state */
    this->mValue = !this->mHit ? Epsilon : this->mValue;
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_COUNTING_GRID_CELL_HPP */
