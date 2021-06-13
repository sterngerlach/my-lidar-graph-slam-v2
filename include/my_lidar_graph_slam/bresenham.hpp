
/* bresenham.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_BRESENHAM_HPP
#define MY_LIDAR_GRAPH_SLAM_BRESENHAM_HPP

#include <cmath>
#include <vector>

#include "my_lidar_graph_slam/point.hpp"

namespace MyLidarGraphSlam {

/* Execute bresenham algorithm */
template <typename T>
void Bresenham(const Point2D<T>& startIdx,
               const Point2D<T>& endIdx,
               std::vector<Point2D<T>>& indices)
{
    /* Clear the indices */
    indices.clear();

    int deltaX = endIdx.mX - startIdx.mX;
    int deltaY = endIdx.mY - startIdx.mY;
    int stepX = (deltaX < 0) ? -1 : 1;
    int stepY = (deltaY < 0) ? -1 : 1;
    int nextX = startIdx.mX;
    int nextY = startIdx.mY;

    deltaX = std::abs(deltaX * 2);
    deltaY = std::abs(deltaY * 2);

    /* Append the start cell index */
    indices.emplace_back(nextX, nextY);

    /* Execute Bresenham algorithm */
    if (deltaX > deltaY) {
        int err = deltaY - deltaX / 2;

        while (nextX != endIdx.mX) {
            if (err >= 0) {
                nextY += stepY;
                err -= deltaX;
            }
            nextX += stepX;
            err += deltaY;
            indices.emplace_back(nextX, nextY);
        }
    } else {
        int err = deltaX - deltaY / 2;

        while (nextY != endIdx.mY) {
            if (err >= 0) {
                nextX += stepX;
                err -= deltaY;
            }
            nextY += stepY;
            err += deltaX;
            indices.emplace_back(nextX, nextY);
        }
    }
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_BRESENHAM_HPP */
