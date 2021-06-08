
/* grid_map_geometry.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_GEOMETRY_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_GEOMETRY_HPP

#include <cmath>

#include "my_lidar_graph_slam/bounding_box.hpp"
#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/*
 * GridMapGeometry struct represents geometric information of the grid maps
 */
struct GridMapGeometry final
{
    /* Default constructor */
    GridMapGeometry();
    /* Destructor */
    ~GridMapGeometry() = default;

    /* Initialize with the number of grid cells */
    void Initialize(const double resolution,
                    const int rows, const int cols);
    /* Reset to the initial state */
    void Reset();

    /* Copy constructor */
    GridMapGeometry(const GridMapGeometry&) = default;
    /* Copy assignment operator */
    GridMapGeometry& operator=(const GridMapGeometry&) = default;
    /* Move constructor */
    GridMapGeometry(GridMapGeometry&&) noexcept = default;
    /* Move assignment operator */
    GridMapGeometry& operator=(GridMapGeometry&&) noexcept = default;

    /* Resize the grid map */
    void Resize(const int rowMin, const int colMin,
                const int rows, const int cols);

    /* Check if the grid map is empty */
    bool IsEmpty() const;
    /* Check if the index is valid */
    bool IsIndexInside(const int row, const int col) const;
    /* Check if the local coordinate is valid */
    bool IsPointInside(const double posX, const double posY) const;

    /* Convert the grid index to the local coordinate */
    Point2D<double> IndexToPosition(
        const int row, const int col) const;
    /* Convert the local coordinate to the grid index */
    Point2D<int> PositionToIndex(
        const double posX, const double posY) const;
    /* Convert the local coordinate to the grid index in floating-points */
    Point2D<double> PositionToIndexF(
        const double posX, const double posY) const;

    /* Get the bounding box (index range) of the grid map */
    BoundingBox<int> IndexBoundingBox() const;
    /* Get the bounding box (local coordinate range) of the grid map */
    BoundingBox<double> PositionBoundingBox() const;

    /* Compute the distance between two grid cells */
    double Distance(const int row0, const int col0,
                    const int row1, const int col1) const;
    /* Compute the squared distance between two grid cells */
    double SquaredDistance(const int row0, const int col0,
                           const int row1, const int col1) const;

    /* Resolution of the grid map */
    double          mResolution;
    /* Number of the grid cell rows */
    int             mRows;
    /* Number of the grid cell columns */
    int             mCols;
    /* Width of the grid map */
    double          mWidth;
    /* Height of the grid map */
    double          mHeight;
    /* Offset to convert the current grid map coordinate frame
     * to the original grid map coordinate frame */
    Point2D<double> mPosOffset;
};

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_MAP_GEOMETRY_HPP */
