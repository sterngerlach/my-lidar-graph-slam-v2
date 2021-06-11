
/* cost_function_greedy_endpoint.cpp */

#include "my_lidar_graph_slam/mapping/cost_function_greedy_endpoint.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
CostGreedyEndpoint::CostGreedyEndpoint(
    const double mapResolution,
    const double hitAndMissedDist,
    const double occupancyThreshold,
    const int kernelSize,
    const double scalingFactor,
    const double standardDeviation) :
    CostFunction(),
    mMapResolution(mapResolution),
    mHitAndMissedDist(hitAndMissedDist),
    mOccupancyThreshold(occupancyThreshold),
    mKernelSize(kernelSize),
    mStandardDeviation(standardDeviation),
    mVariance(standardDeviation * standardDeviation),
    mScalingFactor(scalingFactor),
    mCostLookupTable(nullptr),
    mDefaultCostValue(0.0)
{
    /* Compute the lookup table for the cost values */
    this->SetupLookupTable();
}

/* Calculate cost function based on the squared distance
 * between scan point and its corresponding grid cell */
double CostGreedyEndpoint::Cost(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    double sumCostValue = 0.0;

    const std::size_t numOfScans = scanData->NumOfScans();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Calculate the grid cell index corresponding to the
         * scan point and the missed point */
        Point2D<double> localHitPoint;
        Point2D<double> localMissedPoint;
        scanData->HitAndMissedPoint(
            mapLocalSensorPose, i, this->mHitAndMissedDist,
            localHitPoint, localMissedPoint);

        const Point2D<int> hitIdx =
            gridMap.PositionToIndex(localHitPoint.mX, localHitPoint.mY);
        const Point2D<int> missedIdx =
            gridMap.PositionToIndex(localMissedPoint.mX, localMissedPoint.mY);

        /* Find the best grid cell index from the searching window */
        const double unknownProb = gridMap.UnknownProbability();
        double minCostValue = this->mDefaultCostValue;

        for (int ky = -this->mKernelSize; ky <= this->mKernelSize; ++ky) {
            for (int kx = -this->mKernelSize; kx <= this->mKernelSize; ++kx) {
                const double hitCellProb = gridMap.ProbabilityOr(
                    hitIdx.mY + ky, hitIdx.mX + kx, unknownProb);
                const double missedCellProb = gridMap.ProbabilityOr(
                    missedIdx.mY + ky, missedIdx.mX + kx, unknownProb);

                /* Skip if the grid cell has unknown occupancy probability */
                if (hitCellProb == unknownProb ||
                    missedCellProb == unknownProb)
                    continue;

                /* Skip if the occupancy probability of the grid cell
                 * that is assumed to be hit is less than the threshold or
                 * the occupancy probability of the missed grid cell
                 * is greater than the threshold */
                if (hitCellProb < this->mOccupancyThreshold ||
                    missedCellProb > this->mOccupancyThreshold)
                    continue;

                /* Retrieve the cost value using the lookup table */
                const int idxX = this->mKernelSize + kx;
                const int idxY = this->mKernelSize + ky;
                const int tableIdx = idxY * (2 * this->mKernelSize + 1) + idxX;
                const double costValue = this->mCostLookupTable[tableIdx];

                /* Update the minimum cost for the scan point */
                minCostValue = std::min(minCostValue, costValue);
            }
        }

        /* Add to the cost value, which is proportional to the negative
         * log-likelihood of the observation probability and represents the
         * degree of the mismatch between the laser scan and the grid map */
        sumCostValue += minCostValue;
    }

    /* Apply the scaling factor to the cost value */
    sumCostValue *= this->mScalingFactor;

    return sumCostValue;
}

/* Calculate a gradient vector in a map-local coordinate frame */
Eigen::Vector3d CostGreedyEndpoint::ComputeGradient(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    /* Compute the step */
    const double diffLinear = gridMap.Resolution();
    const double diffAngular = 1e-2;

    /* Compute a gradient of the cost function with respect to sensor pose */
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

/* Calculate a covariance matrix in a map-local coordinate frame */
Eigen::Matrix3d CostGreedyEndpoint::ComputeCovariance(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    /* Approximate a covariance matrix using Laplace approximation
     * Covariance matrix is computed from the inverse of a Hessian matrix
     * of a cost function at the estimated robot pose (optimum point) */
    /* Hessian matrix is then calculated using a Jacobian matrix based on
     * Gauss-Newton approximation */

    /* Calculate the gradient vector */
    const Eigen::Vector3d gradVec =
        this->ComputeGradient(gridMap, scanData, mapLocalSensorPose);

    /* Create a covariance matrix */
    Eigen::Matrix3d covMat = gradVec * gradVec.transpose();

    /* Add a small constant to the diagonal elements */
    covMat(0, 0) += 0.1;
    covMat(1, 1) += 0.1;
    covMat(2, 2) += 0.1;

    return covMat;
}

/* Setup the lookup table for the cost value */
void CostGreedyEndpoint::SetupLookupTable()
{
    /* Allocate the lookup table */
    const int kernelSize = 2 * this->mKernelSize + 1;
    this->mCostLookupTable.reset(new double[kernelSize * kernelSize]);

    /* Compute the cost values in the lookup table */
    for (int ky = -this->mKernelSize; ky <= this->mKernelSize; ++ky) {
        for (int kx = -this->mKernelSize; kx <= this->mKernelSize; ++kx) {
            /* Compute the squared distance using the offsets */
            const double diffX = this->mMapResolution * kx;
            const double diffY = this->mMapResolution * ky;
            const double squaredDist = diffX * diffX + diffY * diffY;

            /* Compute the negative log-likelihood of the observation
             * probability (mismatch between the laser scan and the grid map) */
            const double costValue = -std::exp(
                -0.5 * squaredDist / this->mVariance);

            /* Set the cost value to the lookup table */
            const int idxX = this->mKernelSize + kx;
            const int idxY = this->mKernelSize + ky;
            this->mCostLookupTable[idxY * kernelSize + idxX] = costValue;
        }
    }

    /* Compute the default cost value */
    const double maxDiffX = this->mMapResolution * (this->mKernelSize + 1);
    const double maxDiffY = this->mMapResolution * (this->mKernelSize + 1);
    const double maxSquaredDist = maxDiffX * maxDiffX + maxDiffY * maxDiffY;

    /* Compute the negative log-likelihood of the observation probability */
    this->mDefaultCostValue = -std::exp(
        -0.5 * maxSquaredDist / this->mVariance);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
