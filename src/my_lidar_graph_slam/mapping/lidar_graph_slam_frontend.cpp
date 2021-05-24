
/* lidar_graph_slam_frontend.cpp */

#include "my_lidar_graph_slam/mapping/lidar_graph_slam_frontend.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
FrontendMetrics::FrontendMetrics() :
    mInputScanDataCount(nullptr),
    mProcessCount(nullptr),
    mProcessTime(nullptr),
    mProcessScanTime(nullptr),
    mScanDataSetupTime(nullptr),
    mScanMatchingTime(nullptr),
    mFinalScanMatchingTime(nullptr),
    mDataUpdateTime(nullptr),
    mIntervalTravelDist(nullptr),
    mIntervalAngle(nullptr),
    mIntervalTime(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the counter metrics */
    this->mInputScanDataCount =
        pMetricManager->AddCounter("Frontend.InputScanDataCount");
    this->mProcessCount =
        pMetricManager->AddCounter("Frontend.ProcessCount");
    this->mProcessTime =
        pMetricManager->AddCounter("Frontend.ProcessTime");

    /* Register the distribution metrics */
    this->mProcessScanTime =
        pMetricManager->AddDistribution("Frontend.ProcessScanTime");
    this->mScanDataSetupTime =
        pMetricManager->AddDistribution("Frontend.ScanDataSetupTime");
    this->mScanMatchingTime =
        pMetricManager->AddDistribution("Frontend.ScanMatchingTime");
    this->mFinalScanMatchingTime =
        pMetricManager->AddDistribution("Frontend.FinalScanMatchingTime");
    this->mDataUpdateTime =
        pMetricManager->AddDistribution("Frontend.DataUpdateTime");
    this->mIntervalTravelDist =
        pMetricManager->AddDistribution("Frontend.IntervalTravelDist");
    this->mIntervalAngle =
        pMetricManager->AddDistribution("Frontend.IntervalAngle");
    this->mIntervalTime =
        pMetricManager->AddDistribution("Frontend.IntervalTime");
    this->mNumOfScans =
        pMetricManager->AddDistribution("Frontend.NumOfScans");

    /* Register the value sequence metrics */
    this->mProcessFrame =
        pMetricManager->AddValueSequenceInt("Frontend.ProcessFrame");
}

/* Constructor */
LidarGraphSlamFrontend::LidarGraphSlamFrontend(
    const ScanOutlierFilterPtr& scanOutlierFilter,
    const ScanAccumulatorPtr& scanAccumulator,
    const ScanInterpolatorPtr& scanInterpolator,
    const ScanMatcherPtr& scanMatcher,
    const ScanMatcherPtr& finalScanMatcher,
    const RobotPose2D<double>& initialPose,
    const double updateThresholdTravelDist,
    const double updateThresholdAngle,
    const double updateThresholdTime,
    const double loopDetectionThreshold,
    const double degenerationThreshold,
    const double odometryCovarianceScale,
    const bool fuseOdometryCovariance) :
    mProcessCount(0),
    mScanOutlierFilter(scanOutlierFilter),
    mScanAccumulator(scanAccumulator),
    mScanInterpolator(scanInterpolator),
    mScanMatcher(scanMatcher),
    mFinalScanMatcher(finalScanMatcher),
    mInitialPose(initialPose),
    mLastOdomPose(0.0, 0.0, 0.0),
    mAccumulatedTravelDist(0.0),
    mAccumulatedAngle(0.0),
    mLastMapUpdateOdomPose(0.0, 0.0, 0.0),
    mLastMapUpdateTime(0.0),
    mUpdateThresholdTravelDist(updateThresholdTravelDist),
    mUpdateThresholdAngle(updateThresholdAngle),
    mUpdateThresholdTime(updateThresholdTime),
    mLoopDetectionThreshold(loopDetectionThreshold),
    mLastLoopDetectionDist(0.0),
    mDegenerationThreshold(degenerationThreshold),
    mOdometryCovarianceScale(odometryCovarianceScale),
    mFuseOdometryCovariance(fuseOdometryCovariance),
    mMetrics()
{
    /* Make sure that the scan matcher is valid */
    Assert(this->mScanMatcher != nullptr);
    Assert(this->mFinalScanMatcher != nullptr);
}

/* Process scan data and odometry information */
bool LidarGraphSlamFrontend::ProcessScan(
    LidarGraphSlam* const pParent,
    const Sensor::ScanDataPtr<double>& rawScanData,
    const RobotPose2D<double>& odomPose)
{
    /* Create the timer */
    Metric::Timer outerTimer;

    /* Calculate the relative odometry pose */
    const RobotPose2D<double> relOdomPose = (this->mProcessCount == 0) ?
        RobotPose2D<double>(0.0, 0.0, 0.0) :
        InverseCompound(this->mLastOdomPose, odomPose);

    this->mLastOdomPose = odomPose;

    /* Accumulate the linear and angular distance */
    this->mAccumulatedTravelDist += Distance(relOdomPose);
    this->mAccumulatedAngle += std::fabs(relOdomPose.mTheta);

    /* Accumulate scan data if necessary */
    if (this->mScanAccumulator != nullptr)
        this->mScanAccumulator->AppendScan(rawScanData);

    /* Compute the elapsed time since the last map update */
    const double elapsedTime = (this->mProcessCount == 0) ?
        0.0 : rawScanData->TimeStamp() - this->mLastMapUpdateTime;

    /* Map is updated only when the robot moved more than the specified
     * distance or the specified time has elapsed since the last map update */
    const bool travelDistThreshold =
        this->mAccumulatedTravelDist >= this->mUpdateThresholdTravelDist;
    const bool angleThreshold =
        this->mAccumulatedAngle >= this->mUpdateThresholdAngle;
    const bool timeThreshold =
        elapsedTime >= this->mUpdateThresholdTime;
    const bool isFirstScan = this->mProcessCount == 0;
    const bool mapUpdateNeeded =
        (travelDistThreshold || angleThreshold ||
         timeThreshold || isFirstScan) && (elapsedTime >= 0.0);

    /* Update the metrics */
    this->mMetrics.mInputScanDataCount->Increment();

    if (!mapUpdateNeeded) {
        /* Update the metrics */
        this->mMetrics.mProcessTime->Increment(outerTimer.ElapsedMicro());
        return false;
    }

    /* Interpolate scan data if necessary */
    auto scanData = (this->mScanAccumulator != nullptr) ?
        this->mScanAccumulator->ComputeConcatenatedScan() :
        rawScanData;

    if (this->mScanOutlierFilter != nullptr)
        this->mScanOutlierFilter->RemoveOutliers(scanData);

    if (this->mScanInterpolator != nullptr)
        this->mScanInterpolator->Interpolate(scanData);

    /* Update the pose graph */
    if (this->mProcessCount == 0) {
        /* Create the timer */
        Metric::Timer timer;

        /* Set the initial pose for the first scan */
        RobotPose2D<double> estimatedPose = this->mInitialPose;
        /* Update the pose graph and grid map */
        pParent->AppendFirstNodeAndEdge(estimatedPose, scanData);

        /* Update the metrics and stop the timer */
        this->mMetrics.mDataUpdateTime->Observe(timer.ElapsedMicro());
        timer.Stop();
    } else {
        /* Create the timer */
        Metric::Timer timer;

        /* Wait for the SLAM backend optimization to be done */
        pParent->WaitForOptimization();

        /* Retrieve the latest pose and the latest map */
        RobotPose2D<double> latestScanPose;
        RobotPose2D<double> latestMapPose;
        Point2D<double> latestMapCenterPos;
        GridMapType latestMap;
        pParent->GetLatestData(latestScanPose, latestMap,
                               latestMapPose, latestMapCenterPos);

        /* Update the metrics and restart the timer */
        this->mMetrics.mScanDataSetupTime->Observe(timer.ElapsedMicro());
        timer.Start();

        const RobotPose2D<double> relPoseFromLastMapUpdate =
            InverseCompound(this->mLastMapUpdateOdomPose, odomPose);
        const RobotPose2D<double> initialPose =
            Compound(latestScanPose, relPoseFromLastMapUpdate);

        /* Compute an initial pose in a map-local coordinate */
        const RobotPose2D<double> mapLocalInitialPose =
            InverseCompound(latestMapPose, initialPose);
        /* Construct a new scan matching query */
        const ScanMatchingQuery queryInfo {
            latestMap, latestMapCenterPos,
            scanData, mapLocalInitialPose };
        /* Perform scan matching against the grid map that contains
         * the latest scans and obtain the result */
        const ScanMatchingSummary matchingSummary =
            this->mScanMatcher->OptimizePose(queryInfo);
        /* Make sure that the solution is found */
        Assert(matchingSummary.mPoseFound);

        /* Update the metrics and restart the timer */
        this->mMetrics.mScanMatchingTime->Observe(timer.ElapsedMicro());
        timer.Start();

        /* Perform scan matching using the final scan matcher if needed */
        const ScanMatchingQuery finalQueryInfo {
            latestMap, latestMapCenterPos,
            scanData, matchingSummary.mEstimatedPose };
        /* Perform final scan matching */
        const ScanMatchingSummary finalMatchingSummary =
            this->mFinalScanMatcher->OptimizePose(finalQueryInfo);
        /* Make sure that the solution is found */
        Assert(finalMatchingSummary.mPoseFound);

        /* Update the metrics and restart the timer */
        this->mMetrics.mFinalScanMatchingTime->Observe(timer.ElapsedMicro());
        timer.Start();

        /* Compute the estimated pose in a world coordinate frame */
        const RobotPose2D<double> globalEstimatedPose =
            Compound(latestMapPose, finalMatchingSummary.mEstimatedPose);
        /* Compute the relative pose between the latest node pose
         * `latestScanPose` and the estimated pose `globalEstimatedPose`,
         * the former might have been modified by the loop closure
         * in SLAM backend and thus the above variable `latestScanPose`
         * holds the old value before the loop closure */
        const RobotPose2D<double> scanRelativePose =
            InverseCompound(latestScanPose, globalEstimatedPose);

        /* Convert the covariance matrix to world-coordinate frame */
        const Eigen::Matrix3d scanCovarianceMatrix =
            ConvertCovarianceFromLocalToWorld(
                latestMapPose, finalMatchingSummary.mEstimatedCovariance);

        /* Use the results from scan-matching if degeneration is not present */
        RobotPose2D<double> fusedRelativePose = RobotPose2D<double>::Zero;
        Eigen::Matrix3d fusedCovarianceMatrix = Eigen::Matrix3d::Zero();

        /* Check the degeneration using the pose covariance */
        const bool degenerationDetected =
            this->CheckDegeneration(scanCovarianceMatrix);

        if (degenerationDetected) {
            /* Compute the odometry covariance matrix */
            const Eigen::Matrix3d odomCovarianceMatrix =
                this->ComputeOdometryCovariance(
                    relPoseFromLastMapUpdate, elapsedTime);

            if (this->mFuseOdometryCovariance) {
                /* Use the fused relative pose and covariance matrix */
                this->FuseOdometry(
                    relPoseFromLastMapUpdate, odomCovarianceMatrix,
                    scanRelativePose, scanCovarianceMatrix,
                    fusedRelativePose, fusedCovarianceMatrix);
            } else {
                /* Just use the odometry pose and covariance */
                fusedRelativePose = relPoseFromLastMapUpdate;
                fusedCovarianceMatrix = odomCovarianceMatrix;
            }
        }

        /* Update the pose graph and grid map */
        const RobotPose2D<double>& relativePose = degenerationDetected ?
            fusedRelativePose : scanRelativePose;
        const Eigen::Matrix3d& covarianceMatrix = degenerationDetected ?
            fusedCovarianceMatrix : scanCovarianceMatrix;
        pParent->AppendNodeAndEdge(relativePose, covarianceMatrix, scanData);

        /* Update the metrics and stop the timer */
        this->mMetrics.mDataUpdateTime->Observe(timer.ElapsedMicro());
        timer.Stop();

        /* Get the current accumulated travel distance */
        const double accumTravelDist = pParent->AccumTravelDist();
        /* Check if the loop detection should be performed */
        const bool loopDetectionPerformed =
            (accumTravelDist - this->mLastLoopDetectionDist) >=
            this->mLoopDetectionThreshold;
        /* Update the accumulated travel distance when the last loop
         * detection is performed */
        this->mLastLoopDetectionDist = loopDetectionPerformed ?
            accumTravelDist : this->mLastLoopDetectionDist;

        /* Notify the worker thread for the SLAM backend if the new local map
         * is added to the entire grid map */
        if (loopDetectionPerformed)
            pParent->NotifyBackend();
    }

    /* Update the metrics */
    this->mMetrics.mProcessCount->Increment();
    this->mMetrics.mProcessTime->Increment(outerTimer.ElapsedMicro());
    this->mMetrics.mProcessScanTime->Observe(outerTimer.ElapsedMicro());
    this->mMetrics.mIntervalTravelDist->Observe(this->mAccumulatedTravelDist);
    this->mMetrics.mIntervalAngle->Observe(this->mAccumulatedAngle);
    this->mMetrics.mIntervalTime->Observe(elapsedTime);
    this->mMetrics.mNumOfScans->Observe(scanData->NumOfScans());
    this->mMetrics.mProcessFrame->Observe(
        this->mMetrics.mInputScanDataCount->Value() - 1);

    /* Update miscellaneous parameters */
    this->mProcessCount += 1;
    this->mAccumulatedTravelDist = 0.0;
    this->mAccumulatedAngle = 0.0;
    this->mLastMapUpdateOdomPose = odomPose;
    this->mLastMapUpdateTime = scanData->TimeStamp();

    std::cerr << "Processing frame: " << this->mProcessCount << std::endl;

    return true;
}

/* Check the degeneration */
bool LidarGraphSlamFrontend::CheckDegeneration(
    const Eigen::Matrix3d& poseCovarianceMat) const
{
    /* Compute the eigenvalues using the pose covariance matrix */
    const Eigen::Vector2cd covEigenvalues =
        poseCovarianceMat.block<2, 2>(0, 0).eigenvalues();
    const double minEigen = std::min(
        covEigenvalues[0].real(), covEigenvalues[1].real());
    const double maxEigen = std::max(
        covEigenvalues[0].real(), covEigenvalues[1].real());
    /* Compute the ratio between two eigenvalues */
    const double eigenRatio = maxEigen / minEigen;
    /* Degeneration is detected if the ratio is considerably large */
    return eigenRatio > this->mDegenerationThreshold;
}

/* Compute the odometry covariance */
Eigen::Matrix3d LidarGraphSlamFrontend::ComputeOdometryCovariance(
    const RobotPose2D<double>& relativePose,
    const double elapsedTime) const
{
    /* Compute the translational and rotational velocity */
    const double travelDist = Distance(relativePose);
    const double transVelocity =
        std::max(1e-1, travelDist / elapsedTime);
    const double rotVelocity =
        std::max(1e-1, relativePose.mTheta / elapsedTime);

    Eigen::Matrix3d odometryCovMat = Eigen::Matrix3d::Zero();
    odometryCovMat(0, 0) = transVelocity * transVelocity;
    odometryCovMat(1, 1) = transVelocity * transVelocity;
    odometryCovMat(2, 2) = rotVelocity * rotVelocity;

    return odometryCovMat * this->mOdometryCovarianceScale;
}

/* Fuse the results from the odometry and scan-matching */
void LidarGraphSlamFrontend::FuseOdometry(
    const RobotPose2D<double>& odomRelativePose,
    const Eigen::Matrix3d& odomCovarianceMat,
    const RobotPose2D<double>& scanRelativePose,
    const Eigen::Matrix3d& scanCovarianceMat,
    RobotPose2D<double>& fusedRelativePose,
    Eigen::Matrix3d& fusedCovarianceMat) const
{
    /* Compute the fused covariance matrix */
    const Eigen::Matrix3d inverseOdomCovMat = odomCovarianceMat.inverse();
    const Eigen::Matrix3d inverseScanCovMat = scanCovarianceMat.inverse();
    const Eigen::Matrix3d inverseFusedCovMat =
        inverseOdomCovMat + inverseScanCovMat;
    fusedCovarianceMat = inverseFusedCovMat.inverse();

    /* Normalize the odometry relative pose */
    const double normOdomRelTheta = NormalizeAngle(odomRelativePose.mTheta);
    const double normScanRelTheta = NormalizeAngle(scanRelativePose.mTheta);
    const double diffTheta = normScanRelTheta - normOdomRelTheta;
    const double fixedOdomRelTheta =
        (diffTheta > Pi<double>) ? normOdomRelTheta + 2.0 * Pi<double> :
        (diffTheta < -Pi<double>) ? normOdomRelTheta - 2.0 * Pi<double> :
        normOdomRelTheta;
    const Eigen::Vector3d odomRelPose {
        odomRelativePose.mX, odomRelativePose.mY, fixedOdomRelTheta };
    const Eigen::Vector3d scanRelPose {
        scanRelativePose.mX, scanRelativePose.mY, normScanRelTheta };

    /* Compute the fused pose */
    const Eigen::Vector3d weightedOdomRelPose =
        inverseOdomCovMat * odomRelPose;
    const Eigen::Vector3d weightedScanRelPose =
        inverseScanCovMat * scanRelPose;
    const Eigen::Vector3d fusedRelPose =
        fusedCovarianceMat * (weightedOdomRelPose + weightedScanRelPose);

    fusedRelativePose.mX = fusedRelPose(0);
    fusedRelativePose.mY = fusedRelPose(1);
    fusedRelativePose.mTheta = NormalizeAngle(fusedRelPose(2));
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
