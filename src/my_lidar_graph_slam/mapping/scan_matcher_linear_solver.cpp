
/* scan_matcher_linear_solver.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_linear_solver.hpp"

#include <cassert>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherLinearSolverMetrics::ScanMatcherLinearSolverMetrics(
    const std::string& scanMatcherName) :
    mOptimizationTime(nullptr),
    mDiffTranslation(nullptr),
    mDiffRotation(nullptr),
    mNumOfIterations(nullptr),
    mInitialCost(nullptr),
    mFinalCost(nullptr),
    mNumOfScans(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the distribution metrics */
    this->mOptimizationTime = pMetricManager->AddDistribution(
        scanMatcherName + ".OptimizationTime");
    this->mDiffTranslation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffTranslation");
    this->mDiffRotation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffRotation");

    /* Register the value sequence metrics */
    this->mNumOfIterations = pMetricManager->AddValueSequenceInt(
        scanMatcherName + ".NumOfIterations");
    this->mInitialCost = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".InitialCost");
    this->mFinalCost = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".FinalCost");
    this->mNumOfScans = pMetricManager->AddValueSequenceInt(
        scanMatcherName + ".NumOfScans");
}

/* Constructor */
ScanMatcherLinearSolver::ScanMatcherLinearSolver(
    const std::string& scanMatcherName,
    const int numOfIterationsMax,
    const double convergenceThreshold,
    const double initialLambda,
    const CostFuncPtr& costFunc) :
    ScanMatcher(scanMatcherName),
    mNumOfIterationsMax(numOfIterationsMax),
    mConvergenceThreshold(convergenceThreshold),
    mLambda(initialLambda),
    mCostFunc(nullptr),
    mMetrics(scanMatcherName)
{
    /* Ensure that the cost function is evaluated based on square error */
    this->mCostFunc = std::dynamic_pointer_cast<CostSquareError>(costFunc);
    assert(this->mCostFunc != nullptr);
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherLinearSolver::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Create the timer */
    Metric::Timer timer;

    /* Retrieve the query information */
    const auto& gridMap = queryInfo.mGridMap;
    const auto& scanData = queryInfo.mScanData;
    const RobotPose2D<double>& mapLocalInitialPose =
        queryInfo.mMapLocalInitialPose;

    /* Calculate the sensor pose from the initial robot pose */
    const RobotPose2D<double>& relPose = scanData->RelativeSensorPose();
    const RobotPose2D<double> mapLocalSensorPose =
        Compound(mapLocalInitialPose, relPose);

    /* Compute the initial cost value */
    const double initialCost = this->mCostFunc->Cost(
        gridMap, scanData, mapLocalSensorPose);
    const double normalizedInitialCost = initialCost / scanData->NumOfScans();

    /* Minimum cost and the pose */
    double prevCost = initialCost;
    double cost = std::numeric_limits<double>::max();
    RobotPose2D<double> bestSensorPose = mapLocalSensorPose;
    int numOfIterations = 0;

    while (true) {
        /* Perform one scan matching step */
        bestSensorPose = this->OptimizeStep(gridMap, scanData, bestSensorPose);
        /* Compute the cost value */
        cost = this->mCostFunc->Cost(gridMap, scanData, bestSensorPose);

        /* Stop the optimization if the number of iteration steps
         * exceeded the maximum or the cost converged */
        if (++numOfIterations >= this->mNumOfIterationsMax ||
            std::fabs(prevCost - cost) < this->mConvergenceThreshold)
            break;

        /* Update the damping factor (lambda) */
        if (cost < prevCost)
            this->mLambda = std::max(1e-8, this->mLambda * 0.5);
        else
            this->mLambda = std::min(1e-4, this->mLambda * 2.0);

        prevCost = cost;
    }

    /* Compute the normalized cost value */
    const double normalizedCost = cost / scanData->NumOfScans();
    /* Compute the estimated robot pose in a map-local coordinate frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestSensorPose, relPose);
    /* Calculate the pose covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Update the metrics */
    this->mMetrics.mOptimizationTime->Observe(timer.ElapsedMicro());
    this->mMetrics.mDiffTranslation->Observe(
        Distance(mapLocalInitialPose, estimatedPose));
    this->mMetrics.mDiffRotation->Observe(
        std::abs(mapLocalInitialPose.mTheta - estimatedPose.mTheta));
    this->mMetrics.mNumOfIterations->Observe(numOfIterations);
    this->mMetrics.mInitialCost->Observe(normalizedInitialCost);
    this->mMetrics.mFinalCost->Observe(normalizedCost);
    this->mMetrics.mNumOfScans->Observe(scanData->NumOfScans());

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated pose covariance matrix in a map-local frame */
    return ScanMatchingSummary {
        true, normalizedCost, mapLocalInitialPose,
        estimatedPose, estimatedCovariance };
}

/* Perform one optimization step */
RobotPose2D<double> ScanMatcherLinearSolver::OptimizeStep(
    const GridMapType& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalSensorPose)
{
    /* Compute the Hessian matrix and the residual vector */
    Eigen::Matrix3d hessianMat;
    Eigen::Vector3d residualVec;
    this->mCostFunc->ComputeHessianAndResidual(
        gridMap, scanData, mapLocalSensorPose, hessianMat, residualVec);

    /* Add the damping factor to the diagonals of the Hessian matrix */
    hessianMat(0, 0) += this->mLambda;
    hessianMat(1, 1) += this->mLambda;
    hessianMat(2, 2) += this->mLambda;

    /* Obtain the sensor pose increment */
    const Eigen::Vector3d deltaPose =
        hessianMat.colPivHouseholderQr().solve(residualVec);

    /* Update and return the sensor pose */
    return RobotPose2D<double> { mapLocalSensorPose.mX + deltaPose(0),
                                 mapLocalSensorPose.mY + deltaPose(1),
                                 mapLocalSensorPose.mTheta + deltaPose(2) };
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
