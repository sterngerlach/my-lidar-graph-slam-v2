
/* grid_map_geometry.cpp */

#include "my_lidar_graph_slam/grid_map_new/grid_map_geometry.hpp"

namespace MyLidarGraphSlam {
namespace GridMapNew {

/* Default constructor */
GridMapGeometry::GridMapGeometry() :
    mResolution(0.0),
    mRows(0),
    mCols(0),
    mWidth(0.0),
    mHeight(0.0),
    mPosOffset(0.0, 0.0)
{
}

/* Initialize with the number of grid cells */
void GridMapGeometry::Initialize(const double resolution,
                                 const int rows, const int cols)
{
    Assert(resolution > 0.0);
    Assert(rows > 0);
    Assert(cols > 0);

    this->mResolution = resolution;
    this->mRows = rows;
    this->mCols = cols;
    this->mWidth = resolution * cols;
    this->mHeight = resolution * rows;
    this->mPosOffset = Point2D<double>::Zero;
}

/* Reset to the initial state */
void GridMapGeometry::Reset()
{
    this->mResolution = 0.0;
    this->mRows = 0;
    this->mCols = 0;
    this->mWidth = 0.0;
    this->mHeight = 0.0;
    this->mPosOffset = Point2D<double>::Zero;
}

/* Create the scaled geometry */
GridMapGeometry GridMapGeometry::ScaledGeometry(const int subpixelScale) const
{
    GridMapGeometry scaledGeometry;

    scaledGeometry.mResolution = this->mResolution / subpixelScale;
    scaledGeometry.mRows = this->mRows * subpixelScale;
    scaledGeometry.mCols = this->mCols * subpixelScale;
    scaledGeometry.mWidth = this->mWidth;
    scaledGeometry.mHeight = this->mHeight;
    scaledGeometry.mPosOffset = this->mPosOffset;

    return scaledGeometry;
}

/* Resize the grid map given the bounding box (index range) */
void GridMapGeometry::Resize(const int rowMin, const int colMin,
                             const int rows, const int cols)
{
    Assert(rows > 0);
    Assert(cols > 0);

    this->mRows = rows;
    this->mCols = cols;
    this->mWidth = this->mResolution * cols;
    this->mHeight = this->mResolution * rows;
    this->mPosOffset.mX += this->mResolution * colMin;
    this->mPosOffset.mY += this->mResolution * rowMin;
}

/* Check if the grid map is empty */
bool GridMapGeometry::IsEmpty() const
{
    return (this->mRows == 0 || this->mCols == 0);
}

/* Check if the index is valid */
bool GridMapGeometry::IsIndexInside(const int row, const int col) const
{
    return (row >= 0 && row < this->mRows) &&
           (col >= 0 && col < this->mCols);
}

/* Check if the point (map coordinate) is valid */
bool GridMapGeometry::IsPointInside(const double posX, const double posY) const
{
    /* Compute the map coordinates */
    const double mapX = posX - this->mPosOffset.mX;
    const double mapY = posY - this->mPosOffset.mY;

    return (mapX >= 0.0 && mapX < this->mWidth) &&
           (mapY >= 0.0 && mapY < this->mHeight);
}

/* Convert the grid index to the local coordinate */
Point2D<double> GridMapGeometry::IndexToPosition(
    const int row, const int col) const
{
    /* Compute the map coordinates */
    const double mapX = this->mPosOffset.mX + this->mResolution * col;
    const double mapY = this->mPosOffset.mY + this->mResolution * row;

    return Point2D<double> { mapX, mapY };
}

/* Convert the local coordinate to the grid index */
Point2D<int> GridMapGeometry::PositionToIndex(
    const double posX, const double posY) const
{
    const int col = static_cast<int>(std::floor(
        (posX - this->mPosOffset.mX) / this->mResolution));
    const int row = static_cast<int>(std::floor(
        (posY - this->mPosOffset.mY) / this->mResolution));

    return Point2D<int> { col, row };
}

/* Convert the local coordinate to the grid index in floating-points */
Point2D<double> GridMapGeometry::PositionToIndexF(
    const double posX, const double posY) const
{
    const double col = (posX - this->mPosOffset.mX) / this->mResolution;
    const double row = (posY - this->mPosOffset.mY) / this->mResolution;

    return Point2D<double> { col, row };
}

/* Get the bounding box (index range) of the grid map */
BoundingBox<int> GridMapGeometry::IndexBoundingBox() const
{
    return BoundingBox<int> { 0, 0, this->mCols, this->mRows };
}

/* Get the bounding box (local coordinate range) of the grid map */
BoundingBox<double> GridMapGeometry::PositionBoundingBox() const
{
    return BoundingBox<double> { 0.0, 0.0, this->mWidth, this->mHeight };
}

/* Compute the distance between two grid cells */
double GridMapGeometry::Distance(const int row0, const int col0,
                                 const int row1, const int col1) const
{
    return this->mResolution *
           std::hypot(static_cast<double>(col1 - col0),
                      static_cast<double>(row1 - row0));
}

/* Compute the squared distance between two grid cells */
double GridMapGeometry::SquaredDistance(const int row0, const int col0,
                                        const int row1, const int col1) const
{
    const int cols = (col1 - col0) * (col1 - col0);
    const int rows = (row1 - row0) * (row1 - row0);
    return this->mResolution * this->mResolution * (cols + rows);
}

} /* namespace GridMapNew */
} /* namespace MyLidarGraphSlam */
