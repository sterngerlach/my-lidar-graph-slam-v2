
/* data_types.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_NETWORK_DATA_TYPES_HPP
#define MY_LIDAR_GRAPH_SLAM_NETWORK_DATA_TYPES_HPP

#include <vector>

#include "my_lidar_graph_slam/pose.hpp"

namespace MyLidarGraphSlam {
namespace Network {

/*
 * TimedPose2D struct represents a pose with timestamp
 */
struct TimedPose2D final
{
    /* Timestamp */
    double              mTime;
    /* Robot pose */
    RobotPose2D<double> mPose;
};

/*
 * Scan2D struct represents a single scan with timestamp
 */
struct Scan2D final
{
    /* Timestamp */
    double              mTime;
    /* Sensor pose relative to a robot */
    RobotPose2D<double> mSensorPose;
    /* Minimum range of the scan in meters */
    double              mMinRange;
    /* Maximum range of the scan in meters */
    double              mMaxRange;
    /* Minimum angle of the scan in meters */
    double              mMinAngle;
    /* Maximum angle of the scan in meters */
    double              mMaxAngle;
    /* List of ranges */
    std::vector<double> mRanges;
    /* List of angles */
    std::vector<double> mAngles;
};

/*
 * GridMapParams struct represents parameters used when rendering a grid map
 */
struct GridMapParams final
{
    /* Grid map resolution in meters */
    double mResolution;
    /* Grid block size */
    int    mBlockSize;
    /* Subpixel scale for computing missed grid cell indices */
    int    mSubpixelScale;
    /* Minimum scan range */
    double mMinRange;
    /* Maximum scan range */
    double mMaxRange;
    /* Probability for hit grid cells */
    double mProbabilityHit;
    /* Probability for missed grid cells */
    double mProbabilityMiss;
    /* Odds for hit grid cells */
    double mOddsHit;
    /* Odds for missed grid cells */
    double mOddsMiss;
};

} /* namespace Network */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_NETWORK_DATA_TYPES_HPP */
