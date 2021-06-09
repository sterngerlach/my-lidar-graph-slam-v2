
/* grid_map.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_HPP

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>

#include "my_lidar_graph_slam/grid_map_new/grid_map_interface.hpp"
#include "my_lidar_graph_slam/point.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/*
 * GridMap class represents the occupancy grid map and is comprised of
 * 2D array of grid blocks (small chunks of grid cells) to reduce
 * the memory usage (unused grid blocks do not allocate any heap memory).
 * When creating a new grid map, the origin of the map-local coordinate frame
 * is fixed to the corner of the grid cell at (0, 0) and will not be changed
 */
template <typename T>
class GridMap : public GridMapInterface<
    typename T::ProbabilityType, typename T::ValueType>
{
public:
    /* Type definitions */
    using GridType = T;
    using BaseType = GridMapInterface<
        typename T::ProbabilityType, typename T::ValueType>;

    /* Constructor to create an empty grid map */
    GridMap();
    /* Constructor with the number of rows and columns */
    GridMap(const double resolution, const int blockSize,
            const int desiredRows, const int desiredCols);
    /* Constructor with the width and height */
    GridMap(const double resolution, const int blockSize,
            const double width, const double height);

    /* Destructor */
    ~GridMap() = default;

    /* Copy constructor */
    GridMap(const GridMap& other);
    /* Copy assignment operator */
    GridMap& operator=(const GridMap& other);
    /* Move constructor */
    GridMap(GridMap&& other) noexcept;
    /* Move assignment operator */
    GridMap& operator=(GridMap&& other) noexcept;

    /* Constructor from the grid map with different type of grids */
    template <typename U>
    GridMap(const GridMap<U>& other);
    /* Assignment operator from the grid map with different type of grids */
    template <typename U>
    GridMap& operator=(const GridMap<U>& other);

    /* Initialize with the number of rows and columns */
    void Initialize(const double resolution, const int log2BlockSize,
                    const int desiredRows, const int desiredCols);
    /* Reset to the initial state */
    void Reset();
    /* Reset the grid values to unknown */
    void ResetValues();

    /* Get the unknown probability */
    inline typename T::ProbabilityType UnknownProbability() const override {
        return T::UnknownProbability; }
    /* Get the minimum probability */
    inline typename T::ProbabilityType ProbabilityMin() const override {
        return T::ProbabilityMin; }
    /* Get the maximum probability */
    inline typename T::ProbabilityType ProbabilityMax() const override {
        return T::ProbabilityMax; }

    /* Get the unknown value */
    inline typename T::ValueType UnknownValue() const override {
        return T::UnknownValue; }
    /* Get the minimum value */
    inline typename T::ValueType ValueMin() const override {
        return T::ValueMin; }
    /* Get the maximum value */
    inline typename T::ValueType ValueMax() const override {
        return T::ValueMax; }

    /* Check if the grid cell is allocated on the heap */
    bool IsAllocated(const int row, const int col) const override;

    /* Get the internal value of the grid cell */
    typename T::ValueType Value(
        const int row, const int col) const override;
    /* Get the internal value of the grid cell (without checks) */
    typename T::ValueType ValueUnchecked(
        const int row, const int col) const override;
    /* Get the internal value of the grid cell or return the default value */
    typename T::ValueType ValueOr(
        const int row, const int col,
        const typename T::ValueType value) const override;

    /* Get the probability value of the grid cell */
    typename T::ProbabilityType Probability(
        const int row, const int col) const override;
    /* Get the probability value of the grid cell (without checks) */
    typename T::ProbabilityType ProbabilityUnchecked(
        const int row, const int col) const override;
    /* Get the probability value of the grid cell or return the default value */
    typename T::ProbabilityType ProbabilityOr(
        const int row, const int col,
        const typename T::ProbabilityType prob) const override;

    /* Copy the internal values to the buffer */
    void CopyValues(typename T::ValueType* buffer) const override;
    /* Copy the internal values in the specified region to the buffer */
    void CopyValues(typename T::ValueType* buffer,
                    const BoundingBox<int>& boundingBox) const override;
    /* Copy the internal values as std::uint8_t to the buffer */
    void CopyValuesU8(std::uint8_t* buffer) const override;
    /* Copy the internal values in the specified region as std::uint8_t */
    void CopyValuesU8(std::uint8_t* buffer,
                      const BoundingBox<int>& boundingBox) const override;

    /* Set the internal value of the grid cell */
    void SetValue(const int row, const int col,
                  const typename T::ValueType value);
    /* Set the internal value of the grid cell (without checks) */
    void SetValueUnchecked(const int row, const int col,
                           const typename T::ValueType value);

    /* Set the probability value of the grid cell */
    void SetProbability(const int row, const int col,
                        const typename T::ProbabilityType prob);
    /* Set the probability value of the grid cell (without checks) */
    void SetProbabilityUnchecked(const int row, const int col,
                                 const typename T::ProbabilityType prob);

    /* Fill all grid values with the given internal value */
    void FillValue(const typename T::ValueType value);
    /* Fill all grid values with the given probability value */
    void FillProbability(const typename T::ProbabilityType prob);

    /* Update the grid value given an observation */
    void Update(const int row, const int col,
                const typename T::ObservationType observation);
    /* Update the grid value given an observation (without input checks) */
    void UpdateUnchecked(const int row, const int col,
                         const typename T::ObservationType observation);

    /* Check if the block index is valid */
    bool IsBlockInside(const int row, const int col) const;
    /* Check if the block is allocated */
    bool IsBlockAllocated(const int row, const int col) const;

    /* Convert the grid index to the block index */
    Point2D<int> IndexToBlock(const int row, const int col) const;
    /* Convert the grid index to the block index offset */
    Point2D<int> IndexToBlockOffset(const int row, const int col) const;
    /* Convert the block index to the grid index range */
    BoundingBox<int> BlockToIndexRange(const int row, const int col) const;

    /* Get the constant pointer to the blocks */
    inline const T* Block() const {
        return this->mBlocks.get(); }
    /* Get the constant pointer to the row of blocks */
    inline const T* Block(const int row) const {
        return this->mBlocks.get() + (row * this->mBlockCols); }
    /* Get the constant pointer to the block */
    inline const T* Block(const int row, const int col) const {
        return this->mBlocks.get() + (row * this->mBlockCols) + col; }

    /* Get the mutable pointer to the blocks */
    inline T* Block() {
        return this->mBlocks.get(); }
    /* Get the mutable pointer to the row of blocks */
    inline T* Block(const int row) {
        return this->mBlocks.get() + (row * this->mBlockCols); }
    /* Get the mutable pointer to the block */
    inline T* Block(const int row, const int col) {
        return this->mBlocks.get() + (row * this->mBlockCols) + col; }

    /* Get the base-2 logarithm of the size of the blocks */
    inline int Log2BlockSize() const { return this->mLog2BlockSize; }
    /* Get the size of the blocks */
    inline int BlockSize() const { return this->mBlockSize; }
    /* Get the number of the block rows */
    inline int BlockRows() const { return this->mBlockRows; }
    /* Get the number of the block columns */
    inline int BlockCols() const { return this->mBlockCols; }

public:
    /* Resize the grid map given the bounding box (index range) */
    void Resize(const BoundingBox<int>& boundingBox);
    /* Resize the grid map given the bounding box */
    void Resize(const BoundingBox<double>& boundingBox);
    /* Expand the grid map given the bounding box (index range) */
    void Expand(const BoundingBox<int>& boundingBox);
    /* Expand the grid map given the bounding box */
    void Expand(const BoundingBox<double>& boundingBox);

    /* Crop the grid map (remove the unused blocks and shrink to fit) */
    void Crop();
    /* Compute the bounding box of the cropped grid map */
    BoundingBox<int> CroppedBoundingBox() const;
    /* Compute the bounding box of the cropped grid map in blocks */
    BoundingBox<int> CroppedBoundingBoxInBlocks() const;

private:
    /* Allocate the storage for the blocks */
    void Allocate();
    /* Release the storage for the blocks */
    void Release();

    /* Apply the function for each block */
    void ForEachBlock(std::function<void(T&)> func);
    /* Apply the function to the grid cell */
    void Apply(const int row, const int col,
               std::function<void(T*, int, int)> func);
    /* Apply the function to the grid cell (without checks) */
    void ApplyUnchecked(const int row, const int col,
                        std::function<void(T*, int, int)> func);

    /* Copy the internal values to the buffer */
    template <typename U, typename F>
    void CopyValuesInternal(U* buffer, const BoundingBox<int>& boundingBox,
                            F copyFunc) const;

private:
    /* Base-2 logarithm of the size of the blocks */
    int mLog2BlockSize;
    /* Size of the blocks */
    int mBlockSize;
    /* Number of the block rows */
    int mBlockRows;
    /* Number of the block columns */
    int mBlockCols;
    /* Grid blocks */
    std::unique_ptr<T[]> mBlocks;
};

/* Constructor from the grid map with different type of grids */
template <typename T>
template <typename U>
GridMap<T>::GridMap(const GridMap<U>& other)
{
    *this = other;
}

/* Assignment operator from the grid map with different type of grids */
template <typename T>
template <typename U>
GridMap<T>& GridMap<T>::operator=(const GridMap<U>& other)
{
    /* Release the storage for blocks if not valid */
    if (other.Block() == nullptr) {
        this->Reset();
        return *this;
    }

    /* Reallocate the storage for blocks if not valid */
    if (this->mLog2BlockSize != other.Log2BlockSize() ||
        this->mBlockRows != other.BlockRows() ||
        this->mBlockCols != other.BlockCols()) {
        this->mLog2BlockSize = other.Log2BlockSize();
        this->mBlockSize = other.BlockSize();
        this->mBlockRows = other.BlockRows();
        this->mBlockCols = other.BlockCols();
        this->Allocate();
    }

    /* Copy the blocks, which is possible if the type U is assignable to T */
    const int numOfBlocks = this->mBlockRows * this->mBlockCols;
    std::copy_n(other.Block(), numOfBlocks, this->mBlocks.get());

    /* Copy the geometric information */
    this->mGeometry = other.Geometry();

    return *this;
}

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_HPP */
