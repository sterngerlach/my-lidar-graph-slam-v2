
/* bresenham.cpp */

#include "my_lidar_graph_slam/bresenham.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {

/* Perform the Bresenham algorithm */
void Bresenham(const Point2D<int>& startIdx,
               const Point2D<int>& endIdx,
               std::vector<Point2D<int>>& indices)
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

/* Perform the Bresenham algorithm at subpixel accuracy */
void BresenhamScaled(const Point2D<int>& scaledStartIdx,
                     const Point2D<int>& scaledEndIdx,
                     const int subpixelScale,
                     std::vector<Point2D<int>>& indices)
{
    /* The below code is borrowed from the Google Cartographer source
     * cartographer/mapping/2d/probability_grid_range_data_inserter_2d.{h|cc}
     * cartographer/mapping/internal/2d/ray_to_pixel_mask.{h|cc}
     * and adapted to our source code base */

    /* Order the indices by their X coordinate */
    if (scaledStartIdx.mX > scaledEndIdx.mX) {
        BresenhamScaled(scaledEndIdx, scaledStartIdx, subpixelScale, indices);
        return;
    }

    /* Check that the indices are valid */
    Assert(scaledStartIdx.mX >= 0);
    Assert(scaledStartIdx.mY >= 0);
    Assert(scaledEndIdx.mX >= 0);
    Assert(scaledEndIdx.mY >= 0);

    /* Compute the indices in the original pixel coordinates */
    const Point2D<int> startIdx { scaledStartIdx.mX / subpixelScale,
                                  scaledStartIdx.mY / subpixelScale };
    const Point2D<int> endIdx { scaledEndIdx.mX / subpixelScale,
                                scaledEndIdx.mY / subpixelScale };

    /* Clear the indices */
    indices.clear();

    /* In the following special case, a vertical line in full pixels
     * should be drawn, since `scaledStartIdx` and `scaledEndIdx` have
     * the same full pixel X coordinate */
    if (startIdx.mX == endIdx.mX) {
        const int startY = std::min(startIdx.mY, endIdx.mY);
        const int endY = std::max(startIdx.mY, endIdx.mY);

        Point2D<int> currentIdx { startIdx.mX, startY };
        indices.push_back(currentIdx);

        for (; currentIdx.mY <= endY; ++currentIdx.mY)
            if (indices.back() != currentIdx)
                indices.push_back(currentIdx);

        return;
    }

    const std::int64_t dx = scaledEndIdx.mX - scaledStartIdx.mX;
    const std::int64_t dy = scaledEndIdx.mY - scaledStartIdx.mY;
    const std::int64_t denominator = 2 * subpixelScale * dx;

    /* Insert the current full pixel coordinates */
    Point2D<int> currentIdx = startIdx;
    indices.push_back(currentIdx);

    /* To represent the subpixel centers, the factor of 2 * `subpixelScale`
     * is used as the denominator */
    /* An original full pixel is divided into the 2 * `subpixelScale`
     * square subpixels, and the smallest coordinate of the full pixel is 0,
     * the center of the first subpixel is 1 / (2 * `subpixelScale`), the
     * largest coordinate of the first subpixel is 2 / (2 * `subpixelScale`),
     * and the largest coordinate of the full pixel is 1 */

    /* +-+-+-+ -- 1 = (2 * `subpixelScale`) / (2 * `subpixelScale`)
     * | | | |
     * +-+-+-+
     * | | | |
     * +-+-+-+ -- Top edge of first subpixel = 2 / (2 * `subpixelScale`)
     * | | | | -- Center of first subpixel = 1 / (2 * `subpixelScale`)
     * +-+-+-+ -- 0 = 0 / (2 * `subpixelScale`) */

    /* The center of the subpixel part of `scaledStartIdx.mY` assuming the
     * `denominator`, i.e., `subY` / `denominator` is in (0, 1) */
     std::int64_t subY = (2 * (scaledStartIdx.mY % subpixelScale) + 1) * dx;

    /* The below `firstPixel` denotes the distance from the `scaledStartIdx.mX`
     * to the right full pixel border which is scaled by 2 * `subpixelScale` */
    /* The below `lastPixel` denotes the distance from the left full pixel
     * border to the `scaledEndIdx.mX` which is scaled by 2 * `subpixelScale` */
    const int firstPixel = 2 * subpixelScale -
                           (2 * (scaledStartIdx.mX % subpixelScale) + 1);
    const int lastPixel = 2 * (scaledEndIdx.mX % subpixelScale) + 1;

    /* The full pixel X coordinate of the `scaledEndIdx` */
    const int endX = std::max(startIdx.mX, endIdx.mX);

    /* Move from the `scaledStartIdx` to the next pixel border to the right */
    subY += dy * firstPixel;

    if (dy > 0) {
        /* The following code basically follows the Bresenham algorithm at the
         * subpixel scale (https://stackoverflow.com/questions/41195973) */
        while (true) {
            if (indices.back() != currentIdx)
                indices.push_back(currentIdx);

            while (subY > denominator) {
                subY -= denominator;
                ++currentIdx.mY;
                if (indices.back() != currentIdx)
                    indices.push_back(currentIdx);
            }

            if (subY == denominator) {
                subY -= denominator;
                ++currentIdx.mY;
            }

            ++currentIdx.mX;
            if (currentIdx.mX == endX)
                break;

            /* Move from one pixel border to the next */
            subY += 2 * dy * subpixelScale;
        }

        /* Move from the pixel border on the right to `scaledEndIdx` */
        subY += dy * lastPixel;

        if (indices.back() != currentIdx)
            indices.push_back(currentIdx);

        while (subY > denominator) {
            subY -= denominator;
            ++currentIdx.mY;
            if (indices.back() != currentIdx)
                indices.push_back(currentIdx);
        }

        Assert(subY != denominator);
        Assert(currentIdx.mY == endIdx.mY);

        return;
    } else {
        /* The same applies for the non-ascending in Y coordinates */
        while (true) {
            if (indices.back() != currentIdx)
                indices.push_back(currentIdx);

            while (subY < 0) {
                subY += denominator;
                --currentIdx.mY;
                if (indices.back() != currentIdx)
                    indices.push_back(currentIdx);
            }

            if (subY == 0) {
                subY += denominator;
                --currentIdx.mY;
            }

            ++currentIdx.mX;
            if (currentIdx.mX == endX)
                break;

            /* Move from one pixel border to the next */
            subY += 2 * dy * subpixelScale;
        }

        /* Move from the pixel border on the right to `scaledEndIdx` */
        subY += dy * lastPixel;

        if (indices.back() != currentIdx)
            indices.push_back(currentIdx);

        while (subY < 0) {
            subY += denominator;
            --currentIdx.mY;
            if (indices.back() != currentIdx)
                indices.push_back(currentIdx);
        }

        Assert(subY != 0);
        Assert(currentIdx.mY == endIdx.mY);

        return;
    }
}

} /* namespace MyLidarGraphSlam */
