
/* grid_constant.cpp */

#include <algorithm>

#include "my_lidar_graph_slam/grid_map_new/grid_constant.hpp"
#include "my_lidar_graph_slam/grid_map_new/grid_values.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/* Initialize the lookup table for converting from internal values to
 * corresponding probability values */
const std::vector<double> GridConstant::ValueToProbabilityLookup =
    ComputeValueToProbabilityLookup(UnknownValue, UnknownProbability,
                                    ValueMin, ValueMax,
                                    ProbabilityMin, ProbabilityMax);

/* Initialize the lookup table for converting from internal values to
 * corresponding odds */
const std::vector<double> GridConstant::ValueToOddsLookup =
    ComputeValueToOddsLookup(UnknownValue, UnknownProbability,
                             ValueMin, ValueMax,
                             ProbabilityMin, ProbabilityMax);

/* Copy constructor */
GridConstant::GridConstant(const GridConstant& other) :
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
GridConstant& GridConstant::operator=(const GridConstant& other)
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
GridConstant::GridConstant(GridConstant&& other) noexcept :
    mLog2Size(other.mLog2Size),
    mSize(other.mSize),
    mValues(std::move(other.mValues))
{
}

/* Move assignment operator */
GridConstant& GridConstant::operator=(GridConstant&& other) noexcept
{
    if (this == &other)
        return *this;

    this->mLog2Size = other.mLog2Size;
    this->mSize = other.mSize;
    this->mValues = std::move(other.mValues);

    return *this;
}

/* Initialize with the grid size */
void GridConstant::Initialize(const int log2Size)
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
void GridConstant::Reset()
{
    /* Reset the size of this grid */
    this->mLog2Size = 0;
    this->mSize = 0;
    /* Release the storage for the internal values */
    this->mValues.reset(nullptr);
}

/* Reset the internal values to unknown */
void GridConstant::ResetValues()
{
    this->FillValue(UnknownValue);
}

/* Allocate the storage for the internal values */
void GridConstant::Allocate()
{
    /* Allocate the storage for grid values */
    const int numOfValues = 1 << (this->mLog2Size << 1);
    this->mValues.reset(new std::uint16_t[numOfValues]);
    Assert(this->mValues != nullptr);
}

/* Copy the internal values to the given buffer */
void GridConstant::CopyValues(
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
void GridConstant::CopyValuesU8(
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

/* Convert the internal value to the probability value */
double GridConstant::ValueToProbability(const std::uint16_t value)
{
    return ValueToProbabilityLookup[value];
}

/* Convert the probability value to the internal value */
std::uint16_t GridConstant::ProbabilityToValue(const double prob)
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
