
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

/* Allocate the storage for the internal values */
void GridCounted::Allocate()
{
    /* Allocate the storage for grid values and hit counts */
    const int numOfValues = 1 << (this->mLog2Size << 1);

    this->mValues.reset(new std::uint16_t[numOfValues]);
    this->mHits.reset(new std::uint32_t[numOfValues]);
    this->mCounts.reset(new std::uint32_t[numOfValues]);

    Assert(this->mValues != nullptr);
    Assert(this->mHits != nullptr);
    Assert(this->mCounts != nullptr);
}

/* Release the storage for the internal values */
void GridCounted::Release()
{
    this->mValues.reset(nullptr);
    this->mHits.reset(nullptr);
    this->mCounts.reset(nullptr);
}

/* Reset the internal values to unknown */
void GridCounted::ResetValues()
{
    if (this->IsAllocated()) {
        const int numOfValues = 1 << (this->mLog2Size << 1);
        std::fill_n(this->mValues.get(), numOfValues, UnknownValue);
        std::fill_n(this->mHits.get(), numOfValues, 0U);
        std::fill_n(this->mCounts.get(), numOfValues, 0U);
    }
}

/* Copy the internal values to the given buffer */
void GridCounted::CopyValues(
    std::uint16_t* buffer, const int bufferCols,
    const BoundingBox<int>& boundingBox) const
{
    Assert(boundingBox.mMin.mX >= 0);
    Assert(boundingBox.mMin.mY >= 0);
    Assert(boundingBox.mMax.mX <= this->mSize);
    Assert(boundingBox.mMax.mY <= this->mSize);
    Assert(boundingBox.mMax.mX > boundingBox.mMin.mX);
    Assert(boundingBox.mMax.mY > boundingBox.mMin.mY);

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
void GridCounted::CopyValuesU8(
    std::uint8_t* buffer, const int bufferCols,
    const BoundingBox<int>& boundingBox) const
{
    Assert(boundingBox.mMin.mX >= 0);
    Assert(boundingBox.mMin.mY >= 0);
    Assert(boundingBox.mMax.mX <= this->mSize);
    Assert(boundingBox.mMax.mY <= this->mSize);
    Assert(boundingBox.mMax.mX > boundingBox.mMin.mX);
    Assert(boundingBox.mMax.mY > boundingBox.mMin.mY);

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
