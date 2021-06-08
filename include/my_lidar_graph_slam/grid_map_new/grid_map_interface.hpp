
/* grid_map_interface.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_INTERFACE_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_INTERFACE_HPP

#include <cstdint>

#include "my_lidar_graph_slam/grid_map_new/grid_map_geometry.hpp"
#include "my_lidar_graph_slam/bounding_box.hpp"
#include "my_lidar_graph_slam/point.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/*
 * GridMapInterface class serves methods for grid map accesses
 */
template <typename T, typename U>
class GridMapInterface
{
public:
    /* Constructor */
    GridMapInterface() = default;
    /* Destructor */
    virtual ~GridMapInterface() = default;

    /* Copy constructor */
    GridMapInterface(const GridMapInterface&) = default;
    /* Copy assignment operator */
    GridMapInterface& operator=(const GridMapInterface&) = default;
    /* Move constructor */
    GridMapInterface(GridMapInterface&&) noexcept = default;
    /* Move assignment operator */
    GridMapInterface& operator=(GridMapInterface&&) noexcept = default;

    /* Get the unknown probability */
    virtual T UnknownProbability() const = 0;
    /* Get the minimum probability */
    virtual T ProbabilityMin() const = 0;
    /* Get the maximum probability */
    virtual T ProbabilityMax() const = 0;
    /* Get the unknown value */
    virtual U UnknownValue() const = 0;
    /* Get the minimum value */
    virtual U ValueMin() const = 0;
    /* Get the maximum value */
    virtual U ValueMax() const = 0;

    /* Check if the grid cell is allocated on the heap */
    virtual bool IsAllocated(const int row, const int col) const = 0;

    /* Get the internal value of the grid cell */
    virtual U Value(const int row, const int col) const = 0;
    /* Get the internal value of the grid cell (without checks) */
    virtual U ValueUnchecked(const int row, const int col) const = 0;
    /* Get the internal value of the grid cell or return the default value */
    virtual U ValueOr(const int row, const int col, const U value) const = 0;

    /* Get the probability value of the grid cell */
    virtual T Probability(const int row, const int col) const = 0;
    /* Get the probability value of the grid cell (without checks) */
    virtual T ProbabilityUnchecked(const int row, const int col) const = 0;
    /* Get the probability value of the grid cell or return the default value */
    virtual T ProbabilityOr(const int row, const int col,
                            const T prob) const = 0;

    /* Copy the internal values to the buffer */
    virtual void CopyValues(U* buffer) const = 0;
    /* Copy the internal values in the specified region to the buffer */
    virtual void CopyValues(U* buffer,
                            const BoundingBox<int>& boundingBox) const = 0;
    /* Copy the internal values as std::uint8_t to the buffer */
    virtual void CopyValuesU8(std::uint8_t* buffer) const = 0;
    /* Copy the internal values in the specified region as std::uint8_t */
    virtual void CopyValuesU8(std::uint8_t* buffer,
                              const BoundingBox<int>& boundingBox) const = 0;

    /* Check if the grid map is empty */
    inline bool IsEmpty() const { return this->mGeometry.IsEmpty(); }
    /* Check if the index is valid */
    inline bool IsIndexInside(const int row, const int col) const {
        return this->mGeometry.IsIndexInside(row, col); }
    /* Check if the local coordinate is valid */
    inline bool IsPointInside(const double posX, const double posY) const {
        return this->mGeometry.IsPointInside(posX, posY); }

    /* Convert the grid index to the local coordinate */
    inline Point2D<double> IndexToPosition(
        const int row, const int col) const {
        return this->mGeometry.IndexToPosition(row, col); }
    /* Convert the local coordinate to the grid index */
    inline Point2D<int> PositionToIndex(
        const double posX, const double posY) const {
        return this->mGeometry.PositionToIndex(posX, posY); }
    /* Convert the local coordinate to the grid index in floating-points */
    inline Point2D<double> PositionToIndexF(
        const double posX, const double posY) const {
        return this->mGeometry.PositionToIndexF(posX, posY); }

    /* Get the bounding box (index range) of the grid map */
    inline BoundingBox<int> IndexBoundingBox() const {
        return this->mGeometry.IndexBoundingBox(); }
    /* Get the bounding box (local coordinate range) of the grid map */
    inline BoundingBox<double> PositionBoundingBox() const {
        return this->mGeometry.PositionBoundingBox(); }

    /* Compute the distance between two grid cells */
    inline double Distance(const int row0, const int col0,
                           const int row1, const int col1) const {
        return this->mGeometry.Distance(row0, col0, row1, col1); }
    /* Compute the squared distance between two grid cells */
    inline double SquaredDistance(const int row0, const int col0,
                                  const int row1, const int col1) const {
        return this->mGeometry.SquaredDistance(row0, col0, row1, col1); }

    /* Get the resolution of the grid map */
    inline double Resolution() const { return this->mGeometry.mResolution; }
    /* Get the number of the grid cell rows */
    inline int Rows() const { return this->mGeometry.mRows; }
    /* Get the number of the grid cell columns */
    inline int Cols() const { return this->mGeometry.mCols; }
    /* Get the width of this grid map */
    inline double Width() const { return this->mGeometry.mWidth; }
    /* Get the height of this grid map */
    inline double Height() const { return this->mGeometry.mHeight; }

protected:
    /* Geometry of the grid map */
    GridMapGeometry mGeometry;
};

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_INTERFACE_HPP */
