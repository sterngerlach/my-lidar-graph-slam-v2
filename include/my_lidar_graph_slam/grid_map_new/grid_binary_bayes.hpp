
/* grid_binary_bayes.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_BINARY_BAYES_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_BINARY_BAYES_HPP

#include <cstdint>
#include <memory>
#include <limits>

#include "my_lidar_graph_slam/bounding_box.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {

/*
 * GridBinaryBayes class represents small chunk of grid cells whose
 * occupancy probabilities are updated via binary Bayes filter
 */
class GridBinaryBayes final
{
public:
    /* Type definitions */
    using ProbabilityType = double;
    using ValueType = std::uint16_t;
    using ObservationType = double;

    /* Constructor with the grid size */
    GridBinaryBayes() : mLog2Size(0),
                        mSize(0),
                        mValues(nullptr) { }
    /* Destructor */
    ~GridBinaryBayes() = default;

    /* Copy constructor */
    GridBinaryBayes(const GridBinaryBayes& other);
    /* Copy assignment operator */
    GridBinaryBayes& operator=(const GridBinaryBayes& other);
    /* Move constructor */
    GridBinaryBayes(GridBinaryBayes&& other) noexcept;
    /* Move assignment operator */
    GridBinaryBayes& operator=(GridBinaryBayes&& other) noexcept;

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
    std::uint16_t Value(const int row, const int col) const;
    /* Get the internal value of the grid cell (index is not checked) */
    std::uint16_t ValueUnchecked(const int row, const int col) const;
    /* Get the internal value of the grid cell or return the default value */
    std::uint16_t ValueOr(const int row, const int col,
                          const std::uint16_t value) const;

    /* Get the probability value of the grid cell */
    double Probability(const int row, const int col) const;
    /* Get the probability value of the grid cell (index is not checked) */
    double ProbabilityUnchecked(const int row, const int col) const;
    /* Get the probability value of the grid cell or return the default value */
    double ProbabilityOr(const int row, const int col,
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

    /* Set the internal value of the grid cell */
    void SetValue(const int row, const int col,
                  const std::uint16_t value);
    /* Set the internal value of the grid cell (index is not checked) */
    void SetValueUnchecked(const int row, const int col,
                           const std::uint16_t value);

    /* Set the probability value of the grid cell */
    void SetProbability(const int row, const int col,
                        const double prob);
    /* Set the probability value of the grid cell (index is not checked) */
    void SetProbabilityUnchecked(const int row, const int col,
                                 const double prob);

    /* Fill all grid values with the given internal value */
    void FillValue(const std::uint16_t value);
    /* Fill all grid values with the given probability value */
    void FillProbability(const double prob);

    /* Update the grid value given an observation */
    void Update(const int row, const int col, const double prob);
    /* Update the grid value given an observation (without input checks) */
    void UpdateUnchecked(const int row, const int col, const double prob);

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
    /* Lookup table for converting from internal values to
     * corresponding odds */
    static const std::vector<double> ValueToOddsLookup;

    /* Convert the internal value to the probability value */
    static double ValueToProbability(const std::uint16_t value);
    /* Convert the probability value to the internal value */
    static std::uint16_t ProbabilityToValue(const double prob);

    /* Compute the odds of the probability value */
    static double ProbabilityToOdds(const double prob);
    /* Compute the probability value from the odds */
    static double OddsToProbability(const double odds);

private:
    /* Base-2 logarithm of the size of this grid */
    int mLog2Size;
    /* Size of this grid */
    int mSize;
    /* Grid values */
    std::unique_ptr<std::uint16_t[]> mValues;
};

/* Get the internal value of the grid cell */
std::uint16_t GridBinaryBayes::Value(const int row, const int col) const
{
    Assert(this->IsInside(row, col));
    return this->mValues[(row << this->mLog2Size) + col];
}

/* Get the internal value of the grid cell (index is not checked) */
std::uint16_t GridBinaryBayes::ValueUnchecked(
    const int row, const int col) const
{
    return this->mValues[(row << this->mLog2Size) + col];
}

/* Get the internal value of the grid cell or return the default value */
std::uint16_t GridBinaryBayes::ValueOr(
    const int row, const int col, const std::uint16_t value) const
{
    return this->IsInside(row, col) ?
           this->mValues[(row << this->mLog2Size) + col] : value;
}

/* Get the probability value of the grid cell */
double GridBinaryBayes::Probability(const int row, const int col) const
{
    Assert(this->IsInside(row, col));
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Get the probability value of the grid cell (index is not checked) */
double GridBinaryBayes::ProbabilityUnchecked(const int row, const int col) const
{
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Get the probability value of the grid cell or return the default value */
double GridBinaryBayes::ProbabilityOr(
    const int row, const int col, const double prob) const
{
    if (!this->IsInside(row, col))
        return prob;
    return ValueToProbabilityLookup[
        this->mValues[(row << this->mLog2Size) + col]];
}

/* Copy the internal values to the given buffer */
void GridBinaryBayes::CopyValues(
    std::uint16_t* buffer, const int bufferCols) const
{
    const BoundingBox<int> boundingBox { 0, 0, this->mSize, this->mSize };
    return this->CopyValues(buffer, bufferCols, boundingBox);
}

/* Copy the internal values as std::uint8_t to the given buffer */
void GridBinaryBayes::CopyValuesU8(
    std::uint8_t* buffer, const int bufferCols) const
{
    const BoundingBox<int> boundingBox { 0, 0, this->mSize, this->mSize };
    return this->CopyValuesU8(buffer, bufferCols, boundingBox);
}

/* Set the internal value of the grid cell */
void GridBinaryBayes::SetValue(
    const int row, const int col, const std::uint16_t value)
{
    Assert(this->IsInside(row, col));
    this->mValues[(row << this->mLog2Size) + col] = value;
}

/* Set the internal value of the grid cell (index is not checked) */
void GridBinaryBayes::SetValueUnchecked(
    const int row, const int col, const std::uint16_t value)
{
    this->mValues[(row << this->mLog2Size) + col] = value;
}

/* Set the probability value of the grid cell */
void GridBinaryBayes::SetProbability(
    const int row, const int col, const double prob)
{
    Assert(this->IsInside(row, col));
    this->mValues[(row << this->mLog2Size) + col] = ProbabilityToValue(prob);
}

/* Set the probability value of the grid cell (index is not checked) */
void GridBinaryBayes::SetProbabilityUnchecked(
    const int row, const int col, const double prob)
{
    this->mValues[(row << this->mLog2Size) + col] = ProbabilityToValue(prob);
}

/* Fill all grid values with the given internal value */
void GridBinaryBayes::FillValue(const std::uint16_t value)
{
    const int numOfValues = 1 << (this->mLog2Size << 1);
    std::fill_n(this->Data(), numOfValues, value);
}

/* Fill all grid values with the given probability value */
void GridBinaryBayes::FillProbability(const double prob)
{
    this->FillValue(ProbabilityToValue(prob));
}

/* Update the grid value given an observation */
void GridBinaryBayes::Update(
    const int row, const int col, const double prob)
{
    Assert(this->IsInside(row, col));
    this->UpdateUnchecked(row, col, prob);
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_BINARY_BAYES_HPP */
