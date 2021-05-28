
/* grid_binary_bayes.cpp */

#include <algorithm>

#include "my_lidar_graph_slam/grid_map_new/grid_binary_bayes.hpp"
#include "my_lidar_graph_slam/grid_map_new/grid_values.hpp"

namespace MyLidarGraphSlam {

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

/* Constructor */
GridBinaryBayes::GridBinaryBayes(const int log2Size) :
    BaseType(log2Size),
    mValues(nullptr)
{
    /* Allocate the storage for grid values */
    const int numOfValues = 1 << (this->mLog2Size << 1);
    this->mValues.reset(new std::uint16_t[numOfValues]);
    Assert(this->mValues != nullptr);

    /* Initialize with the unknown values */
    std::fill_n(this->mValues.get(), numOfValues, UnknownValue);
}

/* Copy constructor */
GridBinaryBayes::GridBinaryBayes(const GridBinaryBayes& other) :
    BaseType(other),
    mValues(nullptr)
{
    *this = other;
}

/* Copy assignment operator */
GridBinaryBayes& GridBinaryBayes::operator=(const GridBinaryBayes& other)
{
    if (this == &other)
        return *this;

    /* Reallocate the storage if the size is different */
    if (this->mLog2Size != other.mLog2Size) {
        const int newNumOfValues = 1 << (other.mLog2Size << 1);
        this->mLog2Size = other.mLog2Size;
        this->mSize = other.mSize;
        this->mValues.reset(new std::uint16_t[newNumOfValues]);
        Assert(this->mValues != nullptr);
    }

    /* Copy the grid values */
    const int numOfValues = 1 << (other.mLog2Size << 1);
    std::copy_n(other.mValues.get(), numOfValues, this->mValues.get());

    return *this;
}

/* Move constructor */
GridBinaryBayes::GridBinaryBayes(GridBinaryBayes&& other) noexcept :
    BaseType(std::move(other)),
    mValues(std::move(other.mValues))
{
}

/* Move assignment operator */
GridBinaryBayes& GridBinaryBayes::operator=(GridBinaryBayes&& other) noexcept
{
    if (this == &other)
        return *this;

    BaseType::operator=(std::move(other));
    this->mValues = std::move(other.mValues);

    return *this;
}

/* Copy the internal values to the given buffer */
void GridBinaryBayes::CopyValues(
    std::uint16_t* buffer, const int bufferCols,
    const int rowMin, const int rowMax) const
{
    Assert(rowMin >= 0);
    Assert(rowMax < this->mSize);
    Assert(rowMax >= rowMin);

    const std::uint16_t* srcBuffer = this->Data(rowMin);
    std::uint16_t* dstBuffer = buffer;

    for (int row = rowMin; row < rowMax; ++row) {
        std::copy_n(srcBuffer, this->mSize, dstBuffer);
        srcBuffer += this->mSize;
        dstBuffer += bufferCols;
    }
}

/* Copy the internal values as std::uint8_t to the given buffer */
void GridBinaryBayes::CopyValuesU8(
    std::uint8_t* buffer, const int bufferCols,
    const int rowMin, const int rowMax) const
{
    Assert(rowMin >= 0);
    Assert(rowMax < this->mSize);
    Assert(rowMax >= rowMin);

    auto rawToU8 = [](const std::uint16_t value) {
        return static_cast<std::uint8_t>(value >> 8); };

    const std::uint16_t* srcBuffer = this->Data(rowMin);
    std::uint8_t* dstBuffer = buffer;

    for (int row = rowMin; row < rowMax; ++row) {
        std::transform(srcBuffer, srcBuffer + this->mSize,
                       dstBuffer, rawToU8);
        srcBuffer += this->mSize;
        dstBuffer += bufferCols;
    }
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

} /* namespace MyLidarGraphSlam */
