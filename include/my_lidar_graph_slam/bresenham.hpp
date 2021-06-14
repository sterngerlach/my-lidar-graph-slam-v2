
/* bresenham.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_BRESENHAM_HPP
#define MY_LIDAR_GRAPH_SLAM_BRESENHAM_HPP

#include <cmath>
#include <vector>

#include "my_lidar_graph_slam/point.hpp"

namespace MyLidarGraphSlam {

/* Perform the Bresenham algorithm */
void Bresenham(const Point2D<int>& startIdx,
               const Point2D<int>& endIdx,
               std::vector<Point2D<int>>& indices);

/* Perform the Bresenham algorithm at subpixel accuracy */
void BresenhamScaled(const Point2D<int>& scaledStartIdx,
                     const Point2D<int>& scaledEndIdx,
                     const int subpixelScale,
                     std::vector<Point2D<int>>& indices);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_BRESENHAM_HPP */
