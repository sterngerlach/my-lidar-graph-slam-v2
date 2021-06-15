
/* lidar_graph_slam_frontend.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_FRONTEND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_FRONTEND_HPP

#include <memory>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/scan_accumulator.hpp"
#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_outlier_filter.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct FrontendMetrics
{
    /* Constructor */
    FrontendMetrics();
    /* Destructor */
    ~FrontendMetrics() = default;

    /* Total number of the input scan data */
    Metric::CounterBase*            mInputScanDataCount;
    /* Total number of the processed scan data */
    Metric::CounterBase*            mProcessCount;
    /* Total processing time of the SLAM frontend */
    Metric::CounterBase*            mProcessTime;
    /* Total processing time for the scan data */
    Metric::DistributionBase*       mProcessScanTime;
    /* Total processing time for setting up the scan data */
    Metric::DistributionBase*       mScanDataSetupTime;
    /* Total processing time for the scan matching */
    Metric::DistributionBase*       mScanMatchingTime;
    /* Total processing time for the final scan matching */
    Metric::DistributionBase*       mFinalScanMatchingTime;
    /* Total processing time for updating the grid map and the pose graph */
    Metric::DistributionBase*       mDataUpdateTime;
    /* Accumulated travel distance between the processed scans */
    Metric::DistributionBase*       mIntervalTravelDist;
    /* Difference of the robot pose angle between the processed scans */
    Metric::DistributionBase*       mIntervalAngle;
    /* Time between the processed scans */
    Metric::DistributionBase*       mIntervalTime;
    /* Number of the scan points for each scan data */
    Metric::DistributionBase*       mNumOfScans;
    /* Frame number of the processed scan data */
    Metric::ValueSequenceBase<int>* mProcessFrame;
};

class LidarGraphSlamFrontend
{
public:
    /* Constructor */
    LidarGraphSlamFrontend(
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
        const bool fuseOdometryCovariance);
    /* Destructor */
    ~LidarGraphSlamFrontend() = default;

    /* Copy constructor (disabled) */
    LidarGraphSlamFrontend(const LidarGraphSlamFrontend&) = delete;
    /* Copy assignment operator (disabled) */
    LidarGraphSlamFrontend& operator=(const LidarGraphSlamFrontend&) = delete;
    /* Move constructor */
    LidarGraphSlamFrontend(LidarGraphSlamFrontend&&) = default;
    /* Move assignment operator */
    LidarGraphSlamFrontend& operator=(LidarGraphSlamFrontend&&) = default;

    /* Process scan data and odometry information */
    bool ProcessScan(LidarGraphSlam* const pParent,
                     const Sensor::ScanDataPtr<double>& rawScanData,
                     const RobotPose2D<double>& odomPose);

    /* Check the degeneration */
    bool CheckDegeneration(const Eigen::Matrix3d& poseCovarianceMat) const;

    /* Compute the odometry covariance */
    Eigen::Matrix3d ComputeOdometryCovariance(
        const RobotPose2D<double>& relativePose,
        const double elapsedTime) const;

    /* Fuse the results from the odometry and scan-matching */
    void FuseOdometry(const RobotPose2D<double>& odomRelativePose,
                      const Eigen::Matrix3d& odomCovarianceMat,
                      const RobotPose2D<double>& scanRelativePose,
                      const Eigen::Matrix3d& scanCovarianceMat,
                      RobotPose2D<double>& fusedRelativePose,
                      Eigen::Matrix3d& fusedCovarianceMat) const;

    /* Retrieve the total number of the processed input data */
    inline int ProcessCount() const { return this->mProcessCount; }

private:
    /* The total number of the processed input data */
    int                   mProcessCount;
    /* Scan outlier filter */
    ScanOutlierFilterPtr  mScanOutlierFilter;
    /* Scan accumulator */
    ScanAccumulatorPtr    mScanAccumulator;
    /* Scan interpolator */
    ScanInterpolatorPtr   mScanInterpolator;
    /* First scan matcher */
    ScanMatcherPtr        mScanMatcher;
    /* Final scan matcher that executes the scan matching at sub-pixel
     * accuracy and refines the result returned from the first scan matcher */
    ScanMatcherPtr        mFinalScanMatcher;
    /* Initial pose */
    RobotPose2D<double>   mInitialPose;
    /* Last odometry pose */
    RobotPose2D<double>   mLastOdomPose;
    /* Accumulated travel distance since the last map update */
    double                mAccumulatedTravelDist;
    /* Accumulated angle since the last map update */
    double                mAccumulatedAngle;
    /* Odometry pose at the last map update */
    RobotPose2D<double>   mLastMapUpdateOdomPose;
    /* Time of the last map update */
    double                mLastMapUpdateTime;
    /* Map update threshold for accumulated travel distance */
    const double          mUpdateThresholdTravelDist;
    /* Map update threshold for accumulated angle */
    const double          mUpdateThresholdAngle;
    /* Map update threshold for the elapsed time since the last map update */
    const double          mUpdateThresholdTime;
    /* Travel distance threshold for performing loop detections */
    const double          mLoopDetectionThreshold;
    /* Accumulated travel distance when the last loop detection is performed */
    double                mLastLoopDetectionDist;
    /* Eigenvalues threshold for checking degeneration */
    const double          mDegenerationThreshold;
    /* Scaling factor for the odometry pose covariance */
    const double          mOdometryCovarianceScale;
    /* Flag to deterine whether the covariance should be fused */
    const bool            mFuseOdometryCovariance;
    /* Metrics information */
    FrontendMetrics       mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_FRONTEND_HPP */
