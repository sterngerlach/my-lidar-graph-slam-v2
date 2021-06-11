
/* cost_function_square_error.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class CostSquareError;
using CostSquareErrorPtr = std::shared_ptr<CostSquareError>;

class CostSquareError final : public CostFunction
{
public:
    /* MapValues struct stores the four occupancy probability values
     * at integer coordinates closest to the specified grid cell indices
     * in floating-point, which are later used in bilinear interpolations
     * and gradient (with respect to the robot pose) computations */
    struct MapValues
    {
        /* Constructor */
        MapValues(const double dx, const double dy,
                  const double m00, const double m01,
                  const double m10, const double m11);

        /* Compute the smoothed map value using bilinear interpolation */
        double BilinearInterpolation() const;

        /* Relative position of the specified indices */
        const double mDeltaX;
        const double mDeltaY;
        /* Four neighboring map values */
        const double mValue00;
        const double mValue01;
        const double mValue10;
        const double mValue11;
    };

public:
    /* Constructor */
    CostSquareError(const double covarianceScale);

    /* Destructor */
    ~CostSquareError() = default;

    /* Calculate cost function (mismatch between scan data and map) */
    double Cost(
        const GridMapInterface& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a gradient of the cost function with respect to
     * the sensor pose in map-local coordinate frame */
    Eigen::Vector3d ComputeGradient(
        const GridMapInterface& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a covariance matrix */
    Eigen::Matrix3d ComputeCovariance(
        const GridMapInterface& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Compute a Hessian matrix and a residual vector using
     * a gradient of a smoothed map function with respect to
     * the sensor pose in a map-local coordinate frame */
    void ComputeHessianAndResidual(
        const GridMapInterface& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose,
        Eigen::Matrix3d& hessianMat,
        Eigen::Vector3d& residualVec) const;

    /* Calculate a numerical gradient of the cost function with respect to
     * the sensor pose in map-local coordinate frame */
    Eigen::Vector3d ComputeApproxGrad(
        const GridMapInterface& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose);

    /* Calculate a (normalized) gradient of the smoothed map function
     * based on the bilinear interpolation with respect to the
     * specified map-local position of the map */
    Eigen::Vector2d ComputeScaledMapGradMapPoint(
        const MapValues mapValues) const;

    /* Calculate a gradient of the smoothed map function
     * at the specified scan point with respect to the sensor pose */
    Eigen::Vector3d ComputeScaledMapGradSensorPose(
        const MapValues mapValues,
        const Point2D<double>& rotatedScanPoint) const;

    /* Calculate a gradient of the smoothed map function
     * at the specified scan point with respect to the robot pose */
    Eigen::Vector3d ComputeApproxMapGradSensorPose(
        const GridMapInterface& gridMap,
        const RobotPose2D<double>& mapLocalSensorPose,
        const double scanRange,
        const double scanAngle);

    /* Get the four occupancy probability values at the integer coordinates
     * closest to the specified grid cell indices in floating-point */
    MapValues GetClosestMapValues(
        const GridMapInterface& gridMap,
        const Point2D<double>& gridCellIdx) const;

    /* Calculate the smoothed occupancy probability value
     * using bicubic interpolation */
    double ComputeBicubicValue(
        const GridMapInterface& gridMap,
        const Point2D<double>& gridCellIdx) const;

private:
    /* Scale factor for computing the covariance matrix */
    const double mCovarianceScale;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP */
