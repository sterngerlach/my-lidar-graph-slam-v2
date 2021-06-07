
/* bounding_box.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_BOUNDING_BOX_HPP
#define MY_LIDAR_GRAPH_SLAM_BOUNDING_BOX_HPP

#include <algorithm>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {

/*
 * BoundingBox struct represents the rectangle bounding box defined by
 * the minimum corner position and the maximum corner position
 */
template <typename T>
struct BoundingBox
{
    /* Constructor */
    BoundingBox() = default;

    /* Constructor with the corner positions */
    constexpr BoundingBox(const T xMin, const T yMin,
                          const T xMax, const T yMax) :
        mMin(xMin, yMin), mMax(xMax, yMax) { }
    /* Constructor with the corner positions */
    BoundingBox(const Point2D<T>& posMin,
                const Point2D<T>& posMax) :
        mMin(posMin), mMax(posMax) { }

    /* Destructor */
    ~BoundingBox() = default;

    /* Copy constructor */
    BoundingBox(const BoundingBox&) = default;
    /* Copy assignment operator */
    BoundingBox& operator=(const BoundingBox&) = default;
    /* Move constructor */
    BoundingBox(BoundingBox&&) = default;
    /* Move assignment operator */
    BoundingBox& operator=(BoundingBox&&) = default;

    /* Set the corner positions */
    void Set(const T xMin, const T yMin,
             const T xMax, const T yMax);
    /* Expand the bounding box */
    void Expand(const T xMin, const T yMin,
                const T xMax, const T yMax);
    /* Expand the bounding box to contain the given position */
    void Expand(const T posX, const T posY);
    /* Intersect the bounding box */
    void Intersect(const T xMin, const T yMin,
                   const T xMax, const T yMax);

    /* Compute the width of the region */
    inline T Width() const { return this->mMax.mX - this->mMin.mX; }
    /* Compute the height of the region */
    inline T Height() const { return this->mMax.mY - this->mMin.mY; }

    /* Check if the position is inside the region */
    inline bool IsInside(const T posX, const T posY) const {
        return (this->mMin.mX <= posX && posX < this->mMax.mX) &&
               (this->mMin.mY <= posY && posY < this->mMax.mY); }
    /* Check if the position is inside the region */
    inline bool IsInside(const Point2D<T>& pos) const {
        return this->IsInside(pos.mX, pos.mY); }

    /* Check if the position is inside the region (inclusive) */
    inline bool IsInsideInclusive(const T posX, const T posY) const {
        return (this->mMin.mX <= posX && posX <= this->mMax.mX) &&
               (this->mMin.mY <= posY && posY <= this->mMax.mY); }
    /* Check if the position is inside the region (inclusive) */
    inline bool IsInsideInclusive(const Point2D<T>& pos) const {
        return this->IsInsideInclusive(pos.mX, pos.mY); }

    /* Minimum corner position */
    Point2D<T> mMin;
    /* Maximum corner position */
    Point2D<T> mMax;

    /* Zero bounding box */
    static constexpr BoundingBox Zero { 0, 0, 0, 0 };
};

/* Set the corner positions */
template <typename T>
void BoundingBox<T>::Set(const T xMin, const T yMin,
                         const T xMax, const T yMax)
{
    this->mMin.mX = xMin;
    this->mMin.mY = yMin;
    this->mMax.mX = xMax;
    this->mMax.mY = yMax;
}

/* Expand the bounding box */
template <typename T>
void BoundingBox<T>::Expand(const T xMin, const T yMin,
                            const T xMax, const T yMax)
{
    this->mMin.mX = std::min(this->mMin.mX, xMin);
    this->mMin.mY = std::min(this->mMin.mY, yMin);
    this->mMax.mX = std::max(this->mMax.mX, xMax);
    this->mMax.mY = std::max(this->mMax.mY, yMax);
}

/* Expand the bounding box to contain the given position */
template <typename T>
void BoundingBox<T>::Expand(const T posX, const T posY)
{
    this->mMin.mX = std::min(this->mMin.mX, posX);
    this->mMin.mY = std::min(this->mMin.mY, posY);
    this->mMax.mX = std::max(this->mMax.mX, posX);
    this->mMax.mY = std::max(this->mMax.mY, posY);
}

/* Intersect the bounding box */
template <typename T>
void BoundingBox<T>::Intersect(const T xMin, const T yMin,
                               const T xMax, const T yMax)
{
    this->mMin.mX = std::max(this->mMin.mX, xMin);
    this->mMin.mY = std::max(this->mMin.mY, yMin);
    this->mMax.mX = std::min(this->mMax.mX, xMax);
    this->mMax.mY = std::min(this->mMax.mY, yMax);
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_BOUNDING_BOX_HPP */
