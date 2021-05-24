
/* pose_eigen.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_EIGEN_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_EIGEN_HPP

#include <cmath>
#include <Eigen/Core>

namespace MyLidarGraphSlam {

/* Compounding operator for Eigen::Vector3d */
inline Eigen::Vector3d Compound(const Eigen::Vector3d& startPose,
                                const Eigen::Vector3d& diffPose)
{
    const double sinTheta = std::sin(startPose[2]);
    const double cosTheta = std::cos(startPose[2]);

    const double x = cosTheta * diffPose[0] -
                     sinTheta * diffPose[1] + startPose[0];
    const double y = sinTheta * diffPose[0] +
                     cosTheta * diffPose[1] + startPose[1];
    const double theta = startPose[2] + diffPose[2];

    return Eigen::Vector3d { x, y, theta };
}

/* Inverse compounding operator for Eigen::Vector3d */
inline Eigen::Vector3d InverseCompound(const Eigen::Vector3d& startPose,
                                       const Eigen::Vector3d& endPose)
{
    const double sinTheta = std::sin(startPose[2]);
    const double cosTheta = std::cos(startPose[2]);

    const Eigen::Vector3d diffPose = endPose - startPose;

    const double x = cosTheta * diffPose[0] + sinTheta * diffPose[1];
    const double y = -sinTheta * diffPose[0] + cosTheta * diffPose[1];
    const double theta = diffPose[2];

    return Eigen::Vector3d { x, y, theta };
}

/* Move forward i.e. compute Compound(startPose, diffPose) */
inline Eigen::Vector3d MoveForward(const Eigen::Vector3d& startPose,
                                   const Eigen::Vector3d& diffPose)
{
    /* Just use the compounding operator */
    return Compound(startPose, diffPose);
}

/* Move backward i.e. compute the pose such that
 * Compound(pose, diffPose) is endPose */
inline Eigen::Vector3d MoveBackward(const Eigen::Vector3d& endPose,
                                    const Eigen::Vector3d& diffPose)
{
    const double theta = endPose[2] - diffPose[2];
    const double sinTheta = std::sin(theta);
    const double cosTheta = std::cos(theta);

    const double x = endPose[0] - cosTheta * diffPose[0] +
                     sinTheta * diffPose[1];
    const double y = endPose[1] - sinTheta * diffPose[0] -
                     cosTheta * diffPose[1];

    return Eigen::Vector3d { x, y, theta };
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_EIGEN_HPP */
