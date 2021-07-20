
/* grid_constant.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_CONSTANT_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_CONSTANT_HPP

#include <cstdint>
#include <limits>
#include <memory>

#include "my_lidar_graph_slam/bounding_box.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/*
 * GridConstant class represents small chunk of grid cells whose
 * occupancy probabilities are directly updated
 */
class GridConstant final
{
public:
    /* Type definitions */
    using ProbabilityType = double;
    using ValueType = std::uint16_t;
    using ObservationType = double;

    /* Constructor with the grid size */
    GridConstant() : mLog2Size(0),
                     mSize(0),
                     mValues(nullptr) { }
    /* Destructor */
    ~GridConstant() = default;

    /* Copy constructor */
    GridConstant(const GridConstant& other);
    /* Copy assignment operator */
    GridConstant& operator=(const GridConstant& other);
    /* Move constructor */
    GridConstant(GridConstant&& other) noexcept;
    /* Move assignment operator */
    GridConstant& operator=(GridConstant&& other) noexcept;

    /* Constructor from the different type of grid */
    template <typename U>
    GridConstant(const U& other);
    /* Assignment operator from the different type of grid */
    template <typename U>
    GridConstant& operator=(const U& other);

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

    /* Set the internal value of the grid cell */
    inline void SetValue(const int row, const int col,
                         const std::uint16_t value);
    /* Set the internal value of the grid cell (index is not checked) */
    inline void SetValueUnchecked(const int row, const int col,
                                  const std::uint16_t value);

    /* Set the probability value of the grid cell */
    inline void SetProbability(const int row, const int col,
                               const double prob);
    /* Set the probability value of the grid cell (index is not checked) */
    inline void SetProbabilityUnchecked(const int row, const int col,
                                        const double prob);

    /* Fill all grid values with the given internal value */
    inline void FillValue(const std::uint16_t value);
    /* Fill all grid values with the given probability value */
    inline void FillProbability(const double prob);

    /* Update the grid value given an observation */
    inline void Update(const int row, const int col,
                       const double prob);
    /* Update the grid value given an observation (without input checks) */
    inline void UpdateUnchecked(const int row, const int col,
                                const double prob);

    /* Update the grid value given an odds */
    inline void UpdateOdds(const int /* row */, const int /* col */,
                           const double /* odds */) {
        XAssert(false, "UpdateOdds() is not implemented"); }
    /* Update the grid value given an odds (without input checks) */
    inline void UpdateOddsUnchecked(const int /* row */, const int /* col */,
                                    const double /* prob */) {
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

    /* Convert the internal value to the probability value */
    static double ValueToProbability(const std::uint16_t value);
    /* Convert the probability value to the internal value */
    static std::uint16_t ProbabilityToValue(const double prob);

private:
    /* Lookup table for converting from internal values to
     * corresponding probability values */
    static const std::vector<double> ValueToProbabilityLookup;
    /* Lookup table for converting from internal values to
     * corresponding odds */
    static const std::vector<double> ValueToOddsLookup;

private:
    /* Base-2 logarithm of the size of this grid */
    int mLog2Size;
    /* Size of this grid */
    int mSize;
    /* Grid values */
    std::unique_ptr<std::uint16_t[]> mValues;
};

/* Constructor from the different type of grid */
template <typename U>
GridConstant::GridConstant(const U& other) :
    mLog2Size(other.Log2Size()),
    mSize(other.Size()),
    mValues(nullptr)
{
    if (other.IsAllocated())
        return;

    /* Allocate the storage for the grid values */
    this->Allocate();

    /* Copy the grid values */
    const int numOfValues = 1 << (other.Log2Size() << 1);
    std::copy_n(other.Data(), numOfValues, this->mValues.get());
}

/* Assignment operator from the different type of grid */
template <typename U>
GridConstant& GridConstant::operator=(const U& other)
{
    /* Release the storage if not allocated */
    if (!other.IsAllocated()) {
        this->Reset();
        return *this;
    }

    /* Reallocate the storage if the size is different */
    if (this->mLog2Size != other.Log2Size()) {
        this->mLog2Size = other.Log2Size();
        this->mSize = other.Size();
        this->Allocate();
    }

    /* Copy the grid values */
    const int numOfValues = 1 << (other.Log2Size() << 1);
    std::copy_n(other.Data(), numOfValues, this->mValues.get());

    return *this;
}

/* Get the internal value of the grid cell */
std::uint16_t GridConstant::Value(const int row, const int col) const
{
    Assert(this->IsInside(row, col));
    return this->mValues[(row << this->mLog2Size) + col];
}

/* Get the internal value of the grid cell (index is not checked) */
std::uint16_t GridConstant::ValueUnchecked(
    const int row, const int col) const
{
    return this->mValues[(row << this->mLog2Size) + col];
}

/* Get the internal value of the grid cell or return the default value */
std::uint16_t GridConstant::ValueOr(
    const int row, const int col, const std::uint16_t value) const
{
    return this->IsInside(row, col) ?
           this->mValues[(row << this->mLog2Size) + col] : value;
}

/* Get the probability value of the grid cell */
double GridConstant::Probability(const int row, const int col) const
{
    Assert(this->IsInside(row, col));
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Get the probability value of the grid cell (index is not checked) */
double GridConstant::ProbabilityUnchecked(const int row, const int col) const
{
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Get the probability value of the grid cell or return the default value */
double GridConstant::ProbabilityOr(
    const int row, const int col, const double prob) const
{
    if (!this->IsInside(row, col))
        return prob;
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Set the internal value of the grid cell */
void GridConstant::SetValue(
    const int row, const int col, const std::uint16_t value)
{
    Assert(this->IsInside(row, col));
    this->mValues[(row << this->mLog2Size) + col] = value;
}

/* Set the internal value of the grid cell (index is not checked) */
void GridConstant::SetValueUnchecked(
    const int row, const int col, const std::uint16_t value)
{
    this->mValues[(row << this->mLog2Size) + col] = value;
}

/* Set the probability value of the grid cell */
void GridConstant::SetProbability(
    const int row, const int col, const double prob)
{
    Assert(this->IsInside(row, col));
    this->mValues[(row << this->mLog2Size) + col] = ProbabilityToValue(prob);
}

/* Set the probability value of the grid cell (index is not checked) */
void GridConstant::SetProbabilityUnchecked(
    const int row, const int col, const double prob)
{
    this->mValues[(row << this->mLog2Size) + col] = ProbabilityToValue(prob);
}

/* Fill all grid values with the given internal value */
void GridConstant::FillValue(const std::uint16_t value)
{
    const int numOfValues = 1 << (this->mLog2Size << 1);
    std::fill_n(this->Data(), numOfValues, value);
}

/* Fill all grid values with the given probability value */
void GridConstant::FillProbability(const double prob)
{
    this->FillValue(ProbabilityToValue(prob));
}

/* Update the grid value given an observation */
void GridConstant::Update(
    const int row, const int col, const double prob)
{
    Assert(this->IsInside(row, col));
    this->UpdateUnchecked(row, col, prob);
}

/* Update the grid value given an observation (without input checks) */
void GridConstant::UpdateUnchecked(
    const int row, const int col, const double prob)
{
    this->mValues[(row << this->mLog2Size) + col] = ProbabilityToValue(prob);
}

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_CONSTANT_HPP */
