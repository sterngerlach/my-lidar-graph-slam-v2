
/* grid_counted.cpp */

#include <algorithm>

#include "my_lidar_graph_slam/grid_map_new/grid_counted.hpp"
#include "my_lidar_graph_slam/grid_map_new/grid_values.hpp"

namespace MyLidarGraphSlam {

/* Initialize the lookup table for converting from internal values to
 * corresponding probability values */
const std::vector<double> GridCounted::ValueToProbabilityLookup =
    ComputeValueToProbabilityLookup(UnknownValue, UnknownProbability,
                                    ValueMin, ValueMax,
                                    ProbabilityMin, ProbabilityMax);

/* Constructor */
GridCounted::GridCounted(const int log2Size) :
    BaseType(log2Size),
    mValues(nullptr),
    mHits(nullptr),
    mCounts(nullptr)
{
    /* Allocate the storage for grid values and hit counts */
    const int numOfValues = 1 << (this->mLog2Size << 1);
    this->mValues.reset(new std::uint16_t[numOfValues]);
    this->mHits.reset(new std::uint32_t[numOfValues]);
    this->mCounts.reset(new std::uint32_t[numOfValues]);
    Assert(this->mValues != nullptr);
    Assert(this->mHits != nullptr);
    Assert(this->mCounts != nullptr);

    /* Initialize with the unknown values */
    std::fill_n(this->mValues.get(), numOfValues, UnknownValue);
    /* Initialize with zeros */
    std::fill_n(this->mHits.get(), numOfValues, 0U);
    std::fill_n(this->mCounts.get(), numOfValues, 0U);
}

/* Copy constructor */
GridCounted::GridCounted(const GridCounted& other) :
    BaseType(other),
    mValues(nullptr),
    mHits(nullptr),
    mCounts(nullptr)
{
    *this = other;
}

/* Copy assignment operator */
GridCounted& GridCounted::operator=(const GridCounted& other)
{
    if (this == &other)
        return *this;

    /* Reallocate the storage if the size is different */
    if (this->mLog2Size != other.mLog2Size) {
        const int newNumOfValues = 1 << (other.mLog2Size << 1);
        this->mLog2Size = other.mLog2Size;
        this->mSize = other.mSize;
        this->mValues.reset(new std::uint16_t[newNumOfValues]);
        this->mHits.reset(new std::uint32_t[newNumOfValues]);
        this->mCounts.reset(new std::uint32_t[newNumOfValues]);
        Assert(this->mValues != nullptr);
        Assert(this->mHits != nullptr);
        Assert(this->mCounts != nullptr);
    }

    /* Copy the grid values and hit counts */
    const int numOfValues = 1 << (other.mLog2Size << 1);
    std::copy_n(other.mValues.get(), numOfValues, this->mValues.get());
    std::copy_n(other.mHits.get(), numOfValues, this->mHits.get());
    std::copy_n(other.mCounts.get(), numOfValues, this->mCounts.get());

    return *this;
}

/* Move constructor */
GridCounted::GridCounted(GridCounted&& other) noexcept :
    BaseType(std::move(other)),
    mValues(std::move(other.mValues)),
    mHits(std::move(other.mHits)),
    mCounts(std::move(other.mCounts))
{
}

/* Move assignment operator */
GridCounted& GridCounted::operator=(GridCounted&& other) noexcept
{
    if (this == &other)
        return *this;

    BaseType::operator=(std::move(other));
    this->mValues = std::move(other.mValues);
    this->mHits = std::move(other.mHits);
    this->mCounts = std::move(other.mCounts);

    return *this;
}

/* Copy the internal values to the given buffer */
void GridCounted::CopyValues(
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
void GridCounted::CopyValuesU8(
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
void GridCounted::UpdateUnchecked(
    const int row, const int col, const bool hit)
{
    /* Get the pointer to the counts */
    const std::uint32_t offset = (row << this->mLog2Size) + col;
    std::uint16_t* gridCell = this->mValues.get() + offset;
    std::uint32_t* gridHit = this->mHits.get() + offset;
    std::uint32_t* gridCount = this->mCounts.get() + offset;

    /* Update the counts */
    *gridHit = hit ? (*gridHit + 1) : *gridHit;
    *gridCount = *gridCount + 1;

    /* Update the probability value */
    const double prob = static_cast<double>(*gridHit) /
                        static_cast<double>(*gridCount);

    /* Set the new grid cell value */
    *gridCell = ProbabilityToValue(prob);
}

/* Convert the internal value to the probability value */
double GridCounted::ValueToProbability(const std::uint16_t value)
{
    return ValueToProbabilityLookup[value];
}

/* Convert the probability value to the internal value */
std::uint16_t GridCounted::ProbabilityToValue(const double prob)
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

} /* namespace MyLidarGraphSlam */
