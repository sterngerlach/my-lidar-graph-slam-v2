
/* grid_counted.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_COUNTED_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_COUNTED_HPP

#include <cstdint>
#include <limits>
#include <memory>

#include "my_lidar_graph_slam/bounding_box.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/*
 * GridCounted class represents small chunk of grid cells whose
 * occupancy probabilities are updated by the number of hits and misses
 */
class GridCounted final
{
public:
    /* Type definitions */
    using ProbabilityType = double;
    using ValueType = std::uint16_t;
    using ObservationType = bool;

    /* Constructor with the grid size */
    GridCounted() : mLog2Size(0),
                    mSize(0),
                    mValues(nullptr),
                    mHits(nullptr),
                    mCounts(nullptr) { }
    /* Destructor */
    ~GridCounted() = default;

    /* Copy constructor */
    GridCounted(const GridCounted& other);
    /* Copy assignment operator */
    GridCounted& operator=(const GridCounted& other);
    /* Move constructor */
    GridCounted(GridCounted&& other) noexcept;
    /* Move assignment operator */
    GridCounted& operator=(GridCounted&& other) noexcept;

    /* Initialize with the grid size */
    void Initialize(const int log2Size);
    /* Reset to the initial state */
    void Reset();
    /* Reset the internal values to unknown */
    void ResetValues();

    /* Get the base-2 logarithm of the size of this grid */
    inline int Log2Size() const { return this->mLog2Size; }
    /* Get the size of this grid (2 to the power of `mLog2Size`) */
    inline int Size() const { return this->mSize; }

    /* Check if the index is valid */
    inline bool IsInside(const int row, const int col) const {
        return (row >= 0 && row < this->mSize) &&
               (col >= 0 && col < this->mSize); }
    /* Check if the grid is allocated */
    inline bool IsAllocated() const { return this->mValues != nullptr; }

    /* Get the constant pointer to the storage */
    inline const std::uint16_t* Data() const {
        return this->mValues.get(); }
    /* Get the constant pointer to the row */
    inline const std::uint16_t* Data(const int row) const {
        return this->mValues.get() + (row << this->mLog2Size); }
    /* Get the constant pointer to the grid cell */
    inline const std::uint16_t* Data(const int row, const int col) const {
        return this->mValues.get() + (row << this->mLog2Size) + col; }

    /* Get the mutable pointer to the storage */
    inline std::uint16_t* Data() {
        return this->mValues.get(); }
    /* Get the mutable pointer to the row */
    inline std::uint16_t* Data(const int row) {
        return this->mValues.get() + (row << this->mLog2Size); }
    /* Get the mutable pointer to the grid cell */
    inline std::uint16_t* Data(const int row, const int col) {
        return this->mValues.get() + (row << this->mLog2Size) + col; }

    /* Get the internal value of the grid cell */
    inline std::uint16_t Value(const int row, const int col) const;
    /* Get the internal value of the grid cell (index is not checked) */
    inline std::uint16_t ValueUnchecked(const int row, const int col) const;
    /* Get the internal value of the grid cell or return the default value */
    inline std::uint16_t ValueOr(const int row, const int col,
                                 const std::uint16_t value) const;

    /* Get the probability value of the grid cell */
    inline double Probability(const int row, const int col) const;
    /* Get the probability value of the grid cell (index is not checked) */
    inline double ProbabilityUnchecked(const int row, const int col) const;
    /* Get the probability value of the grid cell or return the default value */
    inline double ProbabilityOr(const int row, const int col,
                                const double prob) const;

    /* Copy the internal values to the given buffer */
    void CopyValues(std::uint16_t* buffer, const int bufferCols) const;
    /* Copy the internal row values to the given buffer */
    void CopyValues(std::uint16_t* buffer, const int bufferCols,
                    const BoundingBox<int>& boundingBox) const;
    /* Copy the internal values as std::uint8_t to the given buffer */
    void CopyValuesU8(std::uint8_t* buffer, const int bufferCols) const;
    /* Copy the internal values as std::uint8_t to the given buffer */
    void CopyValuesU8(std::uint8_t* buffer, const int bufferCols,
                      const BoundingBox<int>& boundingBox) const;
    /* Copy the internal values as std::uint8_t to the given buffer
     * and ensure the 4-byte aligned accesses */
    void CopyValuesU8x4(std::uint32_t* buffer, const int bufferCols) const;
    /* Copy the internal values as std::uint8_t to the given buffer
     * and ensure the 4-byte aligned accesses */
    void CopyValuesU8x4(std::uint32_t* buffer, const int bufferCols,
                        const BoundingBox<int>& boundingBox) const;

    /* Set the internal value of the grid cell (do nothing) */
    void SetValue(const int /* row */, const int /* col */,
                  const std::uint16_t /* value */) {
        XAssert(false, "SetValue() is not implemented"); }
    /* Set the internal value of the grid cell (do nothing) */
    void SetValueUnchecked(const int /* row */, const int /* col */,
                           const std::uint16_t /* value */) {
        XAssert(false, "SetValueUnchecked() is not implemented"); }

    /* Set the probability value of the grid cell (do nothing) */
    void SetProbability(const int /* row */, const int /* col */,
                        const double /* prob */) {
        XAssert(false, "SetProbability() is not implemented"); }
    /* Set the probability value of the grid cell (do nothing) */
    void SetProbabilityUnchecked(const int /* row */, const int /* col */,
                                 const double /* prob */) {
        XAssert(false, "SetProbabilityUnchecked() is not implemented"); }

    /* Fill all grid values with the given internal value (do nothing) */
    void FillValue(const std::uint16_t /* value */) {
        XAssert(false, "FillValue() is not implemented"); }
    /* Fill all grid values with the given probability value (do nothing) */
    void FillProbability(const double /* prob */) {
        XAssert(false, "FillProbability() is not implemented"); }

    /* Update the grid value given an observation (hit or miss) */
    void Update(const int row, const int col, const bool hit);
    /* Update the grid value given an observation (hit or miss) */
    void UpdateUnchecked(const int row, const int col, const bool hit);

    /* Update the grid value given an odds (not implemented) */
    inline void UpdateOdds(const int /* row */, const int /* col */,
                           const bool /* hit */) {
        XAssert(false, "UpdateOdds() is not implemented"); }
    /* Update the grid value given an odds (not implemented) */
    inline void UpdateOddsUnchecked(const int /* row */, const int /* col */,
                                    const bool /* hit */) {
        XAssert(false, "UpdateOddsUnchecked() is not implemented"); }

    /* Inspect the memory usage in bytes */
    std::uint64_t InspectMemoryUsage() const;

private:
    /* Allocate the storage for the internal values */
    void Allocate();

public:
    /* Unknown probability value */
    static constexpr double UnknownProbability = 0.0;
    /* Unknown internal grid value */
    static constexpr std::uint16_t UnknownValue = 0;

    /* Minimum internal value (0 means unknown) */
    static constexpr std::uint16_t ValueMin = 1U;
    /* Maximum internal value */
    static constexpr std::uint16_t ValueMax =
        std::numeric_limits<std::uint16_t>::max();

    /* Minimum probability value */
    static constexpr double ProbabilityMin = 1e-3;
    /* Maximum probability value */
    static constexpr double ProbabilityMax = 1.0 - ProbabilityMin;

private:
    /* Lookup table for converting from internal values to
     * corresponding probability values */
    static const std::vector<double> ValueToProbabilityLookup;

    /* Convert the internal value to the probability value */
    static double ValueToProbability(const std::uint16_t value);
    /* Convert the probability value to the internal value */
    static std::uint16_t ProbabilityToValue(const double prob);

private:
    /* Base-2 logarithm of the size of this grid */
    int mLog2Size;
    /* Size of this grid */
    int mSize;
    /* Grid values */
    std::unique_ptr<std::uint16_t[]> mValues;
    /* Number of the hits */
    std::unique_ptr<std::uint32_t[]> mHits;
    /* Number of the observations */
    std::unique_ptr<std::uint32_t[]> mCounts;
};

/* Get the internal value of the grid cell */
std::uint16_t GridCounted::Value(const int row, const int col) const
{
    Assert(this->IsInside(row, col));
    return this->mValues[(row << this->mLog2Size) + col];
}

/* Get the internal value of the grid cell (index is not checked) */
std::uint16_t GridCounted::ValueUnchecked(
    const int row, const int col) const
{
    return this->mValues[(row << this->mLog2Size) + col];
}

/* Get the internal value of the grid cell or return the default value */
std::uint16_t GridCounted::ValueOr(
    const int row, const int col, const std::uint16_t value) const
{
    return this->IsInside(row, col) ?
           this->mValues[(row << this->mLog2Size) + col] : value;
}

/* Get the probability value of the grid cell */
double GridCounted::Probability(const int row, const int col) const
{
    Assert(this->IsInside(row, col));
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Get the probability value of the grid cell (index is not checked) */
double GridCounted::ProbabilityUnchecked(const int row, const int col) const
{
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Get the probability value of the grid cell or return the default value */
double GridCounted::ProbabilityOr(
    const int row, const int col, const double prob) const
{
    if (!this->IsInside(row, col))
        return prob;
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_COUNTED_HPP */
