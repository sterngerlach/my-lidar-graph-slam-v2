
/* grid_counted.cpp */

#include <algorithm>

#include "my_lidar_graph_slam/grid_map_new/grid_counted.hpp"
#include "my_lidar_graph_slam/grid_map_new/grid_values.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/* Initialize the lookup table for converting from internal values to
 * corresponding probability values */
const std::vector<double> GridCounted::ValueToProbabilityLookup =
    ComputeValueToProbabilityLookup(UnknownValue, UnknownProbability,
                                    ValueMin, ValueMax,
                                    ProbabilityMin, ProbabilityMax);

/* Copy constructor */
GridCounted::GridCounted(const GridCounted& other) :
    mLog2Size(other.mLog2Size),
    mSize(other.mSize),
    mValues(nullptr),
    mHits(nullptr),
    mCounts(nullptr)
{
    if (!other.IsAllocated())
        return;

    /* Allocate the storage for the internal values */
    this->Allocate();

    /* Copy the grid values and hit counts */
    const int numOfValues = 1 << (other.mLog2Size << 1);
    std::copy_n(other.mValues.get(), numOfValues, this->mValues.get());
    std::copy_n(other.mHits.get(), numOfValues, this->mHits.get());
    std::copy_n(other.mCounts.get(), numOfValues, this->mCounts.get());
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
    mLog2Size(other.mLog2Size),
    mSize(other.mSize),
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

    this->mLog2Size = other.mLog2Size;
    this->mSize = other.mSize;
    this->mValues = std::move(other.mValues);
    this->mHits = std::move(other.mHits);
    this->mCounts = std::move(other.mCounts);

    return *this;
}

/* Initialize with the grid size */
void GridCounted::Initialize(const int log2Size)
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
void GridCounted::Reset()
{
    /* Reset the size of this grid */
    this->mLog2Size = 0;
    this->mSize = 0;
    /* Release the storage for the internal values */
    this->mValues.reset(nullptr);
    this->mHits.reset(nullptr);
    this->mCounts.reset(nullptr);
}

/* Reset the internal values to unknown */
void GridCounted::ResetValues()
{
    const int numOfValues = 1 << (this->mLog2Size << 1);
    std::fill_n(this->mValues.get(), numOfValues, UnknownValue);
    std::fill_n(this->mHits.get(), numOfValues, 0U);
    std::fill_n(this->mCounts.get(), numOfValues, 0U);
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

/* Copy the internal values to the given buffer */
void GridCounted::CopyValues(
    std::uint16_t* buffer, const int bufferCols) const
{
    const BoundingBox<int> boundingBox { 0, 0, this->mSize, this->mSize };
    return this->CopyValues(buffer, bufferCols, boundingBox);
}

/* Copy the internal values to the given buffer */
void GridCounted::CopyValues(
    std::uint16_t* buffer, const int bufferCols,
    const BoundingBox<int>& boundingBox) const
{
    const int cols = boundingBox.mMax.mX - boundingBox.mMin.mX;
    const int rowMin = boundingBox.mMin.mY;
    const int rowMax = boundingBox.mMax.mY;

    const std::uint16_t* srcBuffer =
        this->Data(boundingBox.mMin.mY, boundingBox.mMin.mX);
    std::uint16_t* dstBuffer = buffer;

    if (!this->IsAllocated()) {
        for (int row = rowMin; row < rowMax; ++row) {
            std::fill_n(dstBuffer, cols, UnknownValue);
            dstBuffer += bufferCols;
        }
    } else {
        for (int row = rowMin; row < rowMax; ++row) {
            std::copy_n(srcBuffer, cols, dstBuffer);
            srcBuffer += this->mSize;
            dstBuffer += bufferCols;
        }
    }
}

/* Copy the internal values as std::uint8_t to the given buffer */
void GridCounted::CopyValuesU8(
    std::uint8_t* buffer, const int bufferCols) const
{
    const BoundingBox<int> boundingBox { 0, 0, this->mSize, this->mSize };
    return this->CopyValuesU8(buffer, bufferCols, boundingBox);
}

/* Copy the internal values as std::uint8_t to the given buffer */
void GridCounted::CopyValuesU8(
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

    if (!this->IsAllocated()) {
        for (int row = rowMin; row < rowMax; ++row) {
            std::fill_n(dstBuffer, cols, rawToU8(UnknownValue));
            dstBuffer += bufferCols;
        }
    } else {
        for (int row = rowMin; row < rowMax; ++row) {
            std::transform(srcBuffer, srcBuffer + cols, dstBuffer, rawToU8);
            srcBuffer += this->mSize;
            dstBuffer += bufferCols;
        }
    }
}

/* Copy the internal values as std::uint8_t to the given buffer
 * and ensure the 4-byte aligned access */
void GridCounted::CopyValuesU8x4(
    std::uint32_t* buffer, const int bufferCols) const
{
    const BoundingBox<int> boundingBox { 0, 0, this->mSize, this->mSize };
    this->CopyValuesU8x4(buffer, bufferCols, boundingBox);
}

/* Copy the internal values as std::uint8_t to the given buffer
 * and ensure the 4-byte aligned accesses */
void GridCounted::CopyValuesU8x4(
    std::uint32_t* buffer, const int bufferCols,
    const BoundingBox<int>& boundingBox) const
{
    auto rawToU8 = [](const std::uint16_t value) {
        return static_cast<std::uint32_t>(value >> 8); };

    Assert(bufferCols % 4 == 0);
    Assert(boundingBox.mMin.mX % 4 == 0);
    Assert(boundingBox.mMax.mX % 4 == 0);

    const int bufferColsBy4 = bufferCols >> 2;
    const int colsBy4 = boundingBox.Width() >> 2;
    const int rowMin = boundingBox.mMin.mY;
    const int rowMax = boundingBox.mMax.mY;

    const std::uint16_t* srcBuffer =
        this->Data(boundingBox.mMin.mY, boundingBox.mMin.mX);
    std::uint32_t* dstBuffer = buffer;

    if (!this->IsAllocated()) {
        for (int row = rowMin; row < rowMax; ++row) {
            std::fill_n(dstBuffer, colsBy4, 0U);
            dstBuffer += bufferColsBy4;
        }
    } else {
        for (int row = rowMin; row < rowMax; ++row) {
            const std::uint16_t* colBuffer = srcBuffer;
            std::uint32_t* rowBuffer = dstBuffer;

            for (int col4 = 0; col4 < colsBy4; ++col4) {
                const std::uint32_t value0 = rawToU8(colBuffer[0]);
                const std::uint32_t value1 = rawToU8(colBuffer[1]);
                const std::uint32_t value2 = rawToU8(colBuffer[2]);
                const std::uint32_t value3 = rawToU8(colBuffer[3]);
                const std::uint32_t value = (value0) | (value1 << 8) |
                                            (value2 << 16) | (value3 << 24);
                *rowBuffer = value;

                rowBuffer += 1;
                colBuffer += 4;
            }

            srcBuffer += this->mSize;
            dstBuffer += bufferColsBy4;
        }
    }
}

/* Update the grid value given an observation (hit or miss) */
void GridCounted::Update(
    const int row, const int col, const bool hit)
{
    Assert(this->IsInside(row, col));
    this->UpdateUnchecked(row, col, hit);
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

/* Inspect the memory usage in bytes */
std::uint64_t GridCounted::InspectMemoryUsage() const
{
    std::uint64_t memoryUsage = 0;
    const int numOfValues = this->IsAllocated() ?
                            (1 << (this->mLog2Size << 1)) : 0;

    memoryUsage += sizeof(this->mLog2Size) +
                   sizeof(this->mSize) +
                   sizeof(this->mValues) +
                   sizeof(this->mHits) +
                   sizeof(this->mCounts) +
                   sizeof(std::uint16_t) * numOfValues +
                   sizeof(std::uint32_t) * numOfValues +
                   sizeof(std::uint32_t) * numOfValues;

    return memoryUsage;
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

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */
