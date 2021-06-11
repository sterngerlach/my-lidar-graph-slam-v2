
/* cost_function_square_error.cpp */

#include "my_lidar_graph_slam/mapping/cost_function_square_error.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
CostSquareError::MapValues::MapValues(
    const double dx, const double dy,
    const double m00, const double m01,
    const double m10, const double m11) :
    mDeltaX(dx), mDeltaY(dy),
    mValue00(m00), mValue01(m01),
    mValue10(m10), mValue11(m11)
{
    /* Check the relative position */
    Assert(this->mDeltaX >= 0.0 && this->mDeltaX <= 1.0);
    Assert(this->mDeltaY >= 0.0 && this->mDeltaY <= 1.0);
}

/* Compute the smoothed map value using bilinear interpolation */
double CostSquareError::MapValues::BilinearInterpolation() const
{
    const double interpolated =
        this->mDeltaY * (this->mDeltaX * this->mValue11 +
                         (1.0 - this->mDeltaX) * this->mValue01) +
        (1.0 - this->mDeltaY) * (this->mDeltaX * this->mValue10 +
                                 (1.0 - this->mDeltaX) * this->mValue00);

    return interpolated;
}

/* Constructor */
CostSquareError::CostSquareError(
    const double covarianceScale) :
    CostFunction(),
    mCovarianceScale(covarianceScale)
{
}

/* Calculate cost function based on the squared error of the
 * occupancy probability value */
double CostSquareError::Cost(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    double costValue = 0.0;

    const std::size_t numOfScans = scanData->NumOfScans();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Calculate the grid cell index corresponding to the scan point */
        const Point2D<double> localHitPoint =
            scanData->HitPoint(mapLocalSensorPose, i);

        /* Calculate the smoothed occupancy probability value
         * this must be close to 1 since it corresponds to the hit point */
        const Point2D<double> floatIdx =
            gridMap.PositionToIndexF(localHitPoint.mX, localHitPoint.mY);
        const auto mapValues = this->GetClosestMapValues(gridMap, floatIdx);
        const double smoothedValue = mapValues.BilinearInterpolation();
        /* Calculate the squared error */
        const double squaredError = std::pow(1.0 - smoothedValue, 2.0);
        /* Add to the total cost value */
        costValue += squaredError;
    }

    return costValue;
}

/* Calculate a gradient of the cost function with respect to
 * the sensor pose in map-local coordinate frame */
Eigen::Vector3d CostSquareError::ComputeGradient(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    double gradX = 0.0;
    double gradY = 0.0;
    double gradTheta = 0.0;

    const double reciprocalResolution = 1.0 / gridMap.Resolution();

    /* Compute a gradient of the cost function with respect to sensor pose */
    const std::size_t numOfScans = scanData->NumOfScans();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Calculate the grid cell index corresponding to the scan point */
        const Point2D<double> localHitPoint =
            scanData->HitPoint(mapLocalSensorPose, i);
        const Point2D<double> floatingIdx =
            gridMap.PositionToIndexF(localHitPoint.mX, localHitPoint.mY);

        /* Calculate the smoothed map function */
        const auto mapValues = this->GetClosestMapValues(gridMap, floatingIdx);
        const double smoothedMapValue = mapValues.BilinearInterpolation();
        /* Calculate the error */
        const double errorValue = 1.0 - smoothedMapValue;

        /* Compute the rotated scan point
         * ScanData<T>::SensorLocalHitPoint() cannot be used here */
        const Point2D<double> rotatedScanPoint {
            localHitPoint.mX - mapLocalSensorPose.mX,
            localHitPoint.mY - mapLocalSensorPose.mY };
        /* Compute a scaled gradient of the smoothed map function
         * at the scan point with respect to the sensor pose */
        const Eigen::Vector3d scaledMapGrad =
            this->ComputeScaledMapGradSensorPose(mapValues, rotatedScanPoint);
        const Eigen::Vector3d mapGrad = scaledMapGrad * reciprocalResolution;

        /* Update gradients */
        gradX += errorValue * mapGrad(0);
        gradY += errorValue * mapGrad(1);
        gradTheta += errorValue * mapGrad(2);
    }

    gradX *= -2.0;
    gradY *= -2.0;
    gradTheta *= -2.0;

    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Calculate a covariance matrix in a map-local coordinate frame */
Eigen::Matrix3d CostSquareError::ComputeCovariance(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    /* Compute the Hessian matrix (residual vector is not used) */
    Eigen::Matrix3d hessianMat;
    Eigen::Vector3d residualVec;
    this->ComputeHessianAndResidual(
        gridMap, scanData, mapLocalSensorPose, hessianMat, residualVec);
    /* Compute the covariance matrix by scaling the inverse of Hessian */
    const Eigen::Matrix3d covMat =
        hessianMat.inverse() * this->mCovarianceScale;

    return covMat;
}

/* Compute a Hessian matrix and a residual vector using
 * a gradient of a smoothed map function with respect to
 * the sensor pose in a map-local coordinate frame */
void CostSquareError::ComputeHessianAndResidual(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose,
    Eigen::Matrix3d& hessianMat,
    Eigen::Vector3d& residualVec) const
{
    /* Compute an approximated Hessian matrix and a residual vector */
    hessianMat = Eigen::Matrix3d::Zero();
    residualVec = Eigen::Vector3d::Zero();

    const double reciprocalResolution = 1.0 / gridMap.Resolution();

    const std::size_t numOfScans = scanData->NumOfScans();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Compute the grid cell index corresponding to the scan point */
        const Point2D<double> localHitPoint =
            scanData->HitPoint(mapLocalSensorPose, i);
        const Point2D<double> floatingIdx =
            gridMap.PositionToIndexF(localHitPoint.mX, localHitPoint.mY);
        /* Get the map values closest to the floating-point index */
        const auto mapValues = this->GetClosestMapValues(gridMap, floatingIdx);

        /* Evaluate the (scaled) gradient of the smoothed map function
         * at the scan point with respect to the sensor pose */
        const Point2D<double> rotatedScanPoint {
            localHitPoint.mX - mapLocalSensorPose.mX,
            localHitPoint.mY - mapLocalSensorPose.mY };
        const Eigen::Vector3d scaledMapGrad =
            this->ComputeScaledMapGradSensorPose(mapValues, rotatedScanPoint);
        const Eigen::Vector3d mapGrad = scaledMapGrad * reciprocalResolution;

        /* Compute the Hessian matrix for the scan point */
        const Eigen::Matrix3d scanHessianMat = mapGrad * mapGrad.transpose();
        hessianMat += scanHessianMat;

        /* Compute the residual vector for the scan point */
        const double smoothedMapValue = mapValues.BilinearInterpolation();
        const double mapResidual = 1.0 - smoothedMapValue;
        const Eigen::Vector3d scanResidualVec = mapGrad * mapResidual;
        residualVec += scanResidualVec;
    }
}

/* Calculate a numerical gradient of the cost function with respect to
 * the sensor pose in map-local coordinate frame */
Eigen::Vector3d CostSquareError::ComputeApproxGrad(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    /* Compute a gradient of the cost function with respect to sensor pose
     * by numerical gradient (for debugging purpose) */
    const double diffLinear = gridMap.Resolution();
    const double diffAngular = 1e-2;

    const RobotPose2D<double> deltaX { diffLinear, 0.0, 0.0 };
    const RobotPose2D<double> deltaY { 0.0, diffLinear, 0.0 };
    const RobotPose2D<double> deltaTheta { 0.0, 0.0, diffAngular };

    const auto costValue = [&](const RobotPose2D<double>& pose) {
        return this->Cost(gridMap, scanData, pose); };

    const double diffCostX = costValue(mapLocalSensorPose + deltaX) -
                             costValue(mapLocalSensorPose - deltaX);
    const double diffCostY = costValue(mapLocalSensorPose + deltaY) -
                             costValue(mapLocalSensorPose - deltaY);
    const double diffCostTheta = costValue(mapLocalSensorPose + deltaTheta) -
                                 costValue(mapLocalSensorPose - deltaTheta);

    /* Calculate gradients */
    const double gradX = 0.5 * diffCostX / diffLinear;
    const double gradY = 0.5 * diffCostY / diffLinear;
    const double gradTheta = 0.5 * diffCostTheta / diffAngular;

    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Calculate a (normalized) gradient of the smoothed map function
 * based on the bilinear interpolation with respect to the
 * specified map-local position of the map */
Eigen::Vector2d CostSquareError::ComputeScaledMapGradMapPoint(
    const MapValues mapValues) const
{
    /* Refer to the Equation (5) and (6) in the following paper:
     * Stefan Kohlbrecher, Oskar von Stryk, Johannes Meyer and Uwe Klingauf.
     * "A Flexible and Scalable SLAM System with Full 3D Motion Estimation,"
     * in the Proceedings of the IEEE International Symposium on Safety,
     * Security and Rescue Robotics (SSRR), 2011. */

    /* Divide by the grid map resolution to the following values
     * to get the actual gradient */
    const double scaledGradX =
        mapValues.mDeltaY * (mapValues.mValue11 - mapValues.mValue01) +
        (1.0 - mapValues.mDeltaY) * (mapValues.mValue10 - mapValues.mValue00);
    const double scaledGradY =
        mapValues.mDeltaX * (mapValues.mValue11 - mapValues.mValue10) +
        (1.0 - mapValues.mDeltaX) * (mapValues.mValue01 - mapValues.mValue00);

    return Eigen::Vector2d { scaledGradX, scaledGradY };
}

/* Calculate a gradient of the smoothed map function
 * at the specified scan point with respect to the robot pose */
Eigen::Vector3d CostSquareError::ComputeScaledMapGradSensorPose(
    const MapValues mapValues,
    const Point2D<double>& rotatedScanPoint) const
{
    /* Compute a gradient of the smoothed occupancy probability value
     * with respect to the scan point */
    const Eigen::Vector2d scaledMapGrad =
        this->ComputeScaledMapGradMapPoint(mapValues);

    /* Compute a scaled gradient of the smoothed occupancy probability value
     * at the scan point with respect to the robot pose */
    /* Divide by the grid map resolution to the following values
     * to get the actual gradient */
    const double gradX = scaledMapGrad(0);
    const double gradY = scaledMapGrad(1);
    const double gradTheta = -rotatedScanPoint.mY * scaledMapGrad(0) +
                              rotatedScanPoint.mX * scaledMapGrad(1);

    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Calculate a gradient of the smoothed map function
 * at the specified scan point with respect to the robot pose */
Eigen::Vector3d CostSquareError::ComputeApproxMapGradSensorPose(
    const GridMapInterface& gridMap,
    const RobotPose2D<double>& mapLocalSensorPose,
    const double scanRange,
    const double scanAngle)
{
    /* Compute a smoothed occupancy probability value
     * given a sensor pose and a scan data */
    const auto mapValue = [&](const RobotPose2D<double>& pose) {
        const double cosTheta = std::cos(pose.mTheta + scanAngle);
        const double sinTheta = std::sin(pose.mTheta + scanAngle);
        const Point2D<double> localHitPoint {
            pose.mX + scanRange * cosTheta,
            pose.mY + scanRange * sinTheta };
        const Point2D<double> floatingIdx =
            gridMap.PositionToIndexF(localHitPoint.mX, localHitPoint.mY);
        const auto mapValues = this->GetClosestMapValues(gridMap, floatingIdx);
        return mapValues.BilinearInterpolation();
    };

    const double diffLinear = gridMap.Resolution();
    const double diffAngular = 1e-2;

    const RobotPose2D<double> deltaX { diffLinear, 0.0, 0.0 };
    const RobotPose2D<double> deltaY { 0.0, diffLinear, 0.0 };
    const RobotPose2D<double> deltaTheta { 0.0, 0.0, diffAngular };

    const double diffMapX = mapValue(mapLocalSensorPose + deltaX) -
                            mapValue(mapLocalSensorPose - deltaX);
    const double diffMapY = mapValue(mapLocalSensorPose + deltaY) -
                            mapValue(mapLocalSensorPose - deltaY);
    const double diffMapTheta = mapValue(mapLocalSensorPose + deltaTheta) -
                                mapValue(mapLocalSensorPose - deltaTheta);

    /* Calculate gradients */
    const double gradX = 0.5 * diffMapX / diffLinear;
    const double gradY = 0.5 * diffMapY / diffLinear;
    const double gradTheta = 0.5 * diffMapTheta / diffAngular;

    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Get the four occupancy probability values at the integer coordinates
 * closest to the specified grid cell indices in floating-point */
CostSquareError::MapValues CostSquareError::GetClosestMapValues(
    const GridMapInterface& gridMap,
    const Point2D<double>& gridCellIdx) const
{
    /* Obtain the closest integer coordinates */
    const double x0 = std::floor(gridCellIdx.mX);
    const double y0 = std::floor(gridCellIdx.mY);
    const double dx = gridCellIdx.mX - x0;
    const double dy = gridCellIdx.mY - y0;

    /* Clamp the integer coordinates, since the grid cell index
     * could be out-of-bounds */
    const int xc0 = std::max(static_cast<int>(x0), 0);
    const int yc0 = std::max(static_cast<int>(y0), 0);
    const int xc1 = std::min(xc0 + 1, gridMap.Cols() - 1);
    const int yc1 = std::min(yc0 + 1, gridMap.Rows() - 1);

    /* Obtain the occupancy probability values at four integer coordinates */
    const double m00 = gridMap.ProbabilityOr(yc0, xc0, 0.5);
    const double m01 = gridMap.ProbabilityOr(yc1, xc0, 0.5);
    const double m10 = gridMap.ProbabilityOr(yc0, xc1, 0.5);
    const double m11 = gridMap.ProbabilityOr(yc1, xc1, 0.5);

    return MapValues { dx, dy, m00, m01, m10, m11 };
}

/* Calculate the smoothed occupancy probability value
 * using bicubic interpolation */
double CostSquareError::ComputeBicubicValue(
    const GridMapInterface& gridMap,
    const Point2D<double>& gridCellIdx) const
{
    /* Interpolation kernel function */
    auto h = [](double t) -> double {
        const double at = std::abs(t);

        if (at <= 1.0) {
            const double at3 = std::pow(at, 3.0);
            const double at2 = std::pow(at, 2.0);
            return (at3 - 2.0 * at2 + 1.0);
        } else if (at <= 2.0) {
            const double at3 = std::pow(at, 3.0);
            const double at2 = std::pow(at, 2.0);
            return (-at3 + 5.0 * at2 - 8.0 * at + 4.0);
        }

        return 0.0;
    };

    /* Occupancy grid value function */
    auto f = [&gridMap](double x, double y) -> double {
        /* If the specified grid cell index is out of bounds,
         * the value of the first/last row/column grid cell is returned */
        const int xc = std::clamp(static_cast<int>(x), 0, gridMap.Cols() - 1);
        const int yc = std::clamp(static_cast<int>(y), 0, gridMap.Rows() - 1);

        /* Default value is returned if the grid cell is not yet allocated */
        /* If the grid cell is allocated but not yet observed,
         * the unknown probability (0) is returned */
        return gridMap.ProbabilityOr(yc, xc, gridMap.UnknownProbability());
    };

    /* Perform bicubic interpolation */
    const double x = gridCellIdx.mX;
    const double y = gridCellIdx.mY;
    const double floorX = std::floor(x);
    const double floorY = std::floor(y);

    const double x1 = 1.0 + x - floorX;
    const double x2 = x - floorX;
    const double x3 = floorX + 1.0 - x;
    const double x4 = floorX + 2.0 - x;

    const double y1 = 1.0 + y - floorY;
    const double y2 = y - floorY;
    const double y3 = floorY + 1.0 - y;
    const double y4 = floorY + 2.0 - y;

    const Eigen::Vector4d vecX { h(x1), h(x2), h(x3), h(x4) };
    const Eigen::Vector4d vecY { h(y1), h(y2), h(y3), h(y4) };

    Eigen::Matrix4d matValue;
    matValue << f(x - x1, y - y1), f(x - x1, y - y2),
                f(x - x1, y + y3), f(x - x1, y + y4),
                f(x - x2, y - y1), f(x - x2, y - y2),
                f(x - x2, y + y3), f(x - x2, y + y4),
                f(x + x3, y - y1), f(x + x3, y - y2),
                f(x + x3, y + y3), f(x + x3, y + y4),
                f(x + x4, y - y1), f(x + x4, y - y2),
                f(x + x4, y + y3), f(x + x4, y + y4);

    /* Compute the smoothed value */
    const double smoothedValue = vecX.transpose() * matValue * vecY;

    /* Clamp the occupancy value */
    return std::clamp(smoothedValue,
                      gridMap.ProbabilityMin(),
                      gridMap.ProbabilityMax());
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
