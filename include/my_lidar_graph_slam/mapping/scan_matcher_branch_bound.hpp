
/* scan_matcher_branch_bound.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_BRANCH_BOUND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_BRANCH_BOUND_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include <map>
#include <memory>
#include <queue>
#include <vector>

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct ScanMatcherBranchBoundMetrics
{
    /* Constructor */
    ScanMatcherBranchBoundMetrics(const std::string& scanMatcherName);
    /* Destructor */
    ~ScanMatcherBranchBoundMetrics() = default;

    /* Total processing time for setting up the input */
    Metric::ValueSequenceBase<int>*   mInputSetupTime;
    /* Total processing time for the optimization */
    Metric::ValueSequenceBase<int>*   mOptimizationTime;
    /* Distance between the initial pose and the final pose */
    Metric::ValueSequenceBase<float>* mDiffTranslation;
    /* Absolute difference between the initial angle and the final angle */
    Metric::ValueSequenceBase<float>* mDiffRotation;
    /* Size of the search window along the X-axis */
    Metric::ValueSequenceBase<int>*   mWinSizeX;
    /* Size of the search window along the Y-axis */
    Metric::ValueSequenceBase<int>*   mWinSizeY;
    /* Size of the search window along the Theta-axis */
    Metric::ValueSequenceBase<int>*   mWinSizeTheta;
    /* Step size along the X-axis */
    Metric::ValueSequenceBase<float>* mStepSizeX;
    /* Step size along the Y-axis */
    Metric::ValueSequenceBase<float>* mStepSizeY;
    /* Step size along the Theta-axis */
    Metric::ValueSequenceBase<float>* mStepSizeTheta;
    /* Number of the ignored nodes */
    Metric::ValueSequenceBase<int>*   mNumOfIgnoredNodes;
    /* Number of the processed nodes */
    Metric::ValueSequenceBase<int>*   mNumOfProcessedNodes;
    /* Normalized score value of the best solution */
    Metric::ValueSequenceBase<float>* mScoreValue;
    /* Normalized cost value of the best solution */
    Metric::ValueSequenceBase<float>* mCostValue;
    /* Number of the scan points in the given scan */
    Metric::ValueSequenceBase<int>*   mNumOfScans;
};

class ScanMatcherBranchBound final : public ScanMatcher
{
private:
    /*
     * Node struct holds the necessary information for Branch-and-Bound method
     */
    struct Node
    {
        /* Default constructor */
        Node() = default;
        /* Constructor */
        Node(const int x, const int y, const int theta, const int height,
             const double normalizedScore, const double knownRate) :
            mX(x), mY(y), mTheta(theta), mHeight(height),
            mNormalizedScore(normalizedScore), mKnownRate(knownRate) { }

        /* Equality operator */
        inline bool operator==(const Node& other) const
        { return this->mNormalizedScore == other.mNormalizedScore; }
        /* Inequality operator */
        inline bool operator!=(const Node& other) const
        { return !operator==(other); }

        /* Less-than comparison operator (for std::priority_queue) */
        inline bool operator<(const Node& other) const
        { return this->mNormalizedScore < other.mNormalizedScore; }
        /* Greater-than comparison operator */
        inline bool operator>(const Node& other) const
        { return this->mNormalizedScore > other.mNormalizedScore; }
        /* Less-than or equal to comparison operator */
        inline bool operator<=(const Node& other) const
        { return this->mNormalizedScore <= other.mNormalizedScore; }
        /* Greater-than or equal to comparison operator */
        inline bool operator>=(const Node& other) const
        { return this->mNormalizedScore >= other.mNormalizedScore; }

        /* Check if the node is a leaf node */
        inline bool IsLeafNode() const { return this->mHeight == 0; }

        int    mX;
        int    mY;
        int    mTheta;
        int    mHeight;
        double mNormalizedScore;
        double mKnownRate;
    };

public:
    /* Constructor */
    ScanMatcherBranchBound(
        const std::string& scanMatcherName,
        const std::shared_ptr<ScorePixelAccurate>& scoreFunc,
        const CostFuncPtr& costFunc,
        const int nodeHeightMax,
        const double rangeX,
        const double rangeY,
        const double rangeTheta);

    /* Destructor */
    ~ScanMatcherBranchBound() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const GridMap& gridMap,
        const std::vector<ConstMap>& precompMaps,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalInitialPose,
        const double normalizedScoreThreshold,
        const double knownRateThreshold) const;

    /* Precompute multiple coarser grid maps for scan matching */
    std::vector<ConstMap> ComputeCoarserMaps(const GridMap& gridMap) const;

private:
    /* Compute the search window step */
    void ComputeSearchStep(const GridMap& gridMap,
                           const Sensor::ScanDataPtr<double>& scanData,
                           double& stepX,
                           double& stepY,
                           double& stepTheta) const;

private:
    /* Pixel-accurate score function */
    std::shared_ptr<ScorePixelAccurate> mScoreFunc;
    /* Cost function */
    CostFuncPtr                         mCostFunc;
    /* Maximum height of the tree used in branch-and-bound */
    const int                           mNodeHeightMax;
    /* Linear (horizontal) size of the searching window */
    const double                        mRangeX;
    /* Linear (vertical) size of the searching window */
    const double                        mRangeY;
    /* Angular size of the searching window */
    const double                        mRangeTheta;
    /* Metrics information */
    ScanMatcherBranchBoundMetrics       mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_BRANCH_BOUND_HPP */
