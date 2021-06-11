
/* grid_binary_bayes.cpp */

#include <algorithm>

#include "my_lidar_graph_slam/grid_map_new/grid_binary_bayes.hpp"
#include "my_lidar_graph_slam/grid_map_new/grid_values.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/* Initialize the lookup table for converting from internal values to
 * corresponding probability values */
const std::vector<double> GridBinaryBayes::ValueToProbabilityLookup =
    ComputeValueToProbabilityLookup(UnknownValue, UnknownProbability,
                                    ValueMin, ValueMax,
                                    ProbabilityMin, ProbabilityMax);

/* Initialize the lookup table for converting from internal values to
 * corresponding odds */
const std::vector<double> GridBinaryBayes::ValueToOddsLookup =
    ComputeValueToOddsLookup(UnknownValue, UnknownProbability,
                             ValueMin, ValueMax,
                             ProbabilityMin, ProbabilityMax);

/* Copy constructor */
GridBinaryBayes::GridBinaryBayes(const GridBinaryBayes& other) :
    mLog2Size(other.mLog2Size),
    mSize(other.mSize),
    mValues(nullptr)
{
    if (!other.IsAllocated())
        return;

    /* Allocate the storage for the internal values */
    this->Allocate();

    /* Copy the grid values */
    const int numOfValues = 1 << (other.mLog2Size << 1);
    std::copy_n(other.mValues.get(), numOfValues, this->mValues.get());
}

/* Copy assignment operator */
GridBinaryBayes& GridBinaryBayes::operator=(const GridBinaryBayes& other)
{
    if (this == &other)
        return *this;

    /* Release the storage if not allocated */
    if (!other.IsAllocated()) {
        this->Reset();
        return *this;
    }

    /* Reallocate the storage if the size is different */
    if (this->mLog2Size != other.mLog2Size) {
        this->mLog2Size = other.mLog2Size;
        this->mSize = other.mSize;
        this->Allocate();
    }

    /* Copy the grid values */
    const int numOfValues = 1 << (other.mLog2Size << 1);
    std::copy_n(other.mValues.get(), numOfValues, this->mValues.get());

    return *this;
}

/* Move constructor */
GridBinaryBayes::GridBinaryBayes(GridBinaryBayes&& other) noexcept :
    mLog2Size(other.mLog2Size),
    mSize(other.mSize),
    mValues(std::move(other.mValues))
{
}

/* Move assignment operator */
GridBinaryBayes& GridBinaryBayes::operator=(GridBinaryBayes&& other) noexcept
{
    if (this == &other)
        return *this;

    this->mLog2Size = other.mLog2Size;
    this->mSize = other.mSize;
    this->mValues = std::move(other.mValues);

    return *this;
}

/* Initialize with the grid size */
void GridBinaryBayes::Initialize(const int log2Size)
{
    /* Make sure that the specified size is positive */
    DebugAssert(log2Size >= 0);

    /* Set the size of this grid */
    this->mLog2Size = log2Size;
    this->mSize = 1 << log2Size;
    /* Allocate the storage for the internal values */
    this->Allocate();
    /* Reset the internal values with unknown */
    this->ResetValues();
}

/* Reset to the initial state */
void GridBinaryBayes::Reset()
{
    /* Reset the size of this grid */
    this->mLog2Size = 0;
    this->mSize = 0;
    /* Release the storage for the internal values */
    this->mValues.reset(nullptr);
}

/* Reset the internal values to unknown */
void GridBinaryBayes::ResetValues()
{
    this->FillValue(UnknownValue);
}

/* Allocate the storage for the internal values */
void GridBinaryBayes::Allocate()
{
    /* Allocate the storage for grid values */
    const int numOfValues = 1 << (this->mLog2Size << 1);
    this->mValues.reset(new std::uint16_t[numOfValues]);
    Assert(this->mValues != nullptr);
}

/* Copy the internal values to the given buffer */
void GridBinaryBayes::CopyValues(
    std::uint16_t* buffer, const int bufferCols) const
{
    const BoundingBox<int> boundingBox { 0, 0, this->mSize, this->mSize };
    return this->CopyValues(buffer, bufferCols, boundingBox);
}

/* Copy the internal values to the given buffer */
void GridBinaryBayes::CopyValues(
    std::uint16_t* buffer, const int bufferCols,
    const BoundingBox<int>& boundingBox) const
{
    const int cols = boundingBox.mMax.mX - boundingBox.mMin.mX;
    const int rowMin = boundingBox.mMin.mY;
    const int rowMax = boundingBox.mMax.mY;

    const std::uint16_t* srcBuffer =
        this->Data(boundingBox.mMin.mY, boundingBox.mMin.mX);
    std::uint16_t* dstBuffer = buffer;

    for (int row = rowMin; row < rowMax; ++row) {
        std::copy_n(srcBuffer, cols, dstBuffer);
        srcBuffer += this->mSize;
        dstBuffer += bufferCols;
    }
}

/* Copy the internal values as std::uint8_t to the given buffer */
void GridBinaryBayes::CopyValuesU8(
    std::uint8_t* buffer, const int bufferCols) const
{
    const BoundingBox<int> boundingBox { 0, 0, this->mSize, this->mSize };
    return this->CopyValuesU8(buffer, bufferCols, boundingBox);
}

/* Copy the internal values as std::uint8_t to the given buffer */
void GridBinaryBayes::CopyValuesU8(
    std::uint8_t* buffer, const int bufferCols,
    const BoundingBox<int>& boundingBox) const
{
    auto rawToU8 = [](const std::uint16_t value) {
        return static_cast<std::uint8_t>(value >> 8); };

    const int cols = boundingBox.mMax.mX - boundingBox.mMin.mX;
    const int rowMin = boundingBox.mMin.mY;
    const int rowMax = boundingBox.mMax.mY;

    const std::uint16_t* srcBuffer =
        this->Data(boundingBox.mMin.mY, boundingBox.mMin.mX);
    std::uint8_t* dstBuffer = buffer;

    for (int row = rowMin; row < rowMax; ++row) {
        std::transform(srcBuffer, srcBuffer + cols, dstBuffer, rawToU8);
        srcBuffer += this->mSize;
        dstBuffer += bufferCols;
    }
}

/* Update the grid value given an observation */
void GridBinaryBayes::Update(
    const int row, const int col, const double prob)
{
    Assert(this->IsInside(row, col));
    this->UpdateUnchecked(row, col, prob);
}

/* Update the grid value given an observation (without input checks) */
void GridBinaryBayes::UpdateUnchecked(
    const int row, const int col, const double prob)
{
    /* Get the pointer to the grid cell value */
    std::uint16_t* gridCell = this->Data(row, col);

    /* Initialize the value if this grid cell is not yet observed */
    if (*gridCell == UnknownValue) {
        *gridCell = ProbabilityToValue(prob);
        return;
    }

    /* Update the probability value via binary Bayes filter */
    const double oldOdds = ValueToOddsLookup[*gridCell];
    const double odds = ProbabilityToOdds(prob);
    const double newProb = OddsToProbability(oldOdds * odds);

    /* Set the new grid cell value */
    *gridCell = ProbabilityToValue(newProb);
}

/* Convert the internal value to the probability value */
double GridBinaryBayes::ValueToProbability(const std::uint16_t value)
{
    return ValueToProbabilityLookup[value];
}

/* Convert the probability value to the internal value */
std::uint16_t GridBinaryBayes::ProbabilityToValue(const double prob)
{
    if (prob == UnknownProbability)
        return UnknownValue;
    if (prob < ProbabilityMin)
        return ValueMin;
    if (prob > ProbabilityMax)
        return ValueMax;

    return MyLidarGraphSlam::ProbabilityToValue(
        prob, ValueMin, ValueMax, ProbabilityMin, ProbabilityMax);
}

/* Compute the odds of the probability value */
double GridBinaryBayes::ProbabilityToOdds(const double prob)
{
    if (prob == UnknownProbability)
        return 1.0;
    if (prob < ProbabilityMin)
        return ProbabilityMin / (1.0 - ProbabilityMin);
    if (prob > ProbabilityMax)
        return ProbabilityMax / (1.0 - ProbabilityMax);

    return MyLidarGraphSlam::ProbabilityToOdds(prob);
}

/* Compute the probability value from the odds */
double GridBinaryBayes::OddsToProbability(const double odds)
{
    if (odds < 0.0)
        return UnknownProbability;

    /* Compute the probability value from the odds */
    const double prob = MyLidarGraphSlam::OddsToProbability(odds);
    /* Clamp the probability value */
    const double clamped = std::clamp(prob, ProbabilityMin, ProbabilityMax);

    return clamped;
}

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */
