
/* grid.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_HPP

#include <vector>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {

/*
 * Grid class represents small chunk of grid cells inside grid maps
 */
template <typename T, typename U, typename V>
class Grid
{
public:
    /* Type definitions */
    using ProbabilityType = T;
    using ValueType = U;

    /* Constructor with the grid size */
    explicit Grid(const int log2Size);
    /* Destructor */
    virtual ~Grid() = default;

    /* Copy constructor */
    Grid(const Grid&) = default;
    /* Copy assignment operator */
    Grid& operator=(const Grid&) = default;
    /* Move constructor */
    Grid(Grid&&) noexcept = default;
    /* Move assignment operator */
    Grid& operator=(Grid&&) noexcept = default;

    /* Get the base-2 logarithm of the size of this grid */
    inline int Log2Size() const { return this->mLog2Size; }
    /* Get the size of this grid (2 to the power of `mLog2Size`) */
    inline int Size() const { return this->mSize; }

    /* Check if the index is valid */
    bool IsInside(const int row, const int col) const;

    /* Get the constant pointer to the storage */
    virtual const U* Data() const = 0;
    /* Get the constant pointer to the row */
    virtual const U* Data(const int row) const = 0;
    /* Get the constant pointer to the grid cell */
    virtual const U* Data(const int row, const int col) const = 0;

    /* Get the mutable pointer to the storage */
    virtual U* Data() = 0;
    /* Get the mutable pointer to the row */
    virtual U* Data(const int row) = 0;
    /* Get the mutable pointer to the grid cell */
    virtual U* Data(const int row, const int col) = 0;

    /* Get the internal value of the grid cell */
    virtual U Value(const int row, const int col) const = 0;
    /* Get the internal value of the grid cell (index is not checked) */
    virtual U ValueUnchecked(const int row, const int col) const = 0;
    /* Get the internal value of the grid cell or return the default value */
    virtual U ValueOr(const int row, const int col, const U value) const = 0;

    /* Get the probability value of the grid cell */
    virtual T Probability(const int row, const int col) const = 0;
    /* Get the probability value of the grid cell (index is not checked) */
    virtual T ProbabilityUnchecked(const int row, const int col) const = 0;
    /* Get the probability value of the grid cell or return the default value */
    virtual T ProbabilityOr(const int row, const int col,
                            const T prob) const = 0;

    /* Copy the internal values to the given buffer */
    virtual void CopyValues(U* buffer, const int bufferCols) const = 0;
    /* Copy the internal values to the given buffer */
    virtual void CopyValues(U* buffer, const int bufferCols,
                            const int rowMin, const int rowMax) const = 0;
    /* Copy the internal values as std::uint8_t to the given buffer */
    virtual void CopyValuesU8(std::uint8_t* buffer,
                              const int bufferCols) const = 0;
    /* Copy the internal values as std::uint8_t to the given buffer */
    virtual void CopyValuesU8(std::uint8_t* buffer, const int bufferCols,
                              const int rowMin, const int rowMax) const = 0;

    /* Set the internal value of the grid cell */
    virtual void SetValue(const int row, const int col, const U value) = 0;
    /* Set the internal value of the grid cell (index is not checked) */
    virtual void SetValueUnchecked(const int row, const int col,
                                   const U value) = 0;

    /* Set the probability value of the grid cell */
    virtual void SetProbability(const int row, const int col,
                                const T prob) = 0;
    /* Set the probability value of the grid cell (index is not checked) */
    virtual void SetProbabilityUnchecked(const int row, const int col,
                                         const T prob) = 0;

    /* Fill all grid values with the given internal value */
    virtual void FillValue(const U value) = 0;
    /* Fill all grid values with the given probability value */
    virtual void FillProbability(const T prob) = 0;

    /* Update the grid value given an observation */
    virtual void Update(const int row, const int col,
                        const V observation) = 0;
    /* Update the grid value given an observation (without input checks) */
    virtual void UpdateUnchecked(const int row, const int col,
                                 const V observation) = 0;

public:
    /* Unknown probability value */
    static constexpr T UnknownProbability = static_cast<T>(0);
    /* Unknown internal grid value */
    static constexpr U UnknownValue = static_cast<U>(0);

protected:
    /* Base-2 logarithm of the size of this grid */
    int mLog2Size;
    /* Size of this grid */
    int mSize;
};

/* Constructor with the grid size */
template <typename T, typename U, typename V>
Grid<T, U, V>::Grid(const int log2Size) :
    mLog2Size(log2Size),
    mSize(0)
{
    /* Make sure that the specified size is positive */
    DebugAssert(this->mLog2Size >= 0);
    /* Set the size of this grid */
    this->mSize = 1 << this->mLog2Size;
}

/* Check if the index is valid */
template <typename T, typename U, typename V>
bool Grid<T, U, V>::IsInside(const int row, const int col) const
{
    return (row >= 0 && row < this->mSize) &&
           (col >= 0 && col < this->mSize);
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_Grid_HPP */
