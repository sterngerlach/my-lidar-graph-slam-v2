
/* scan_matcher_branch_bound.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_branch_bound.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherBranchBoundMetrics::ScanMatcherBranchBoundMetrics(
    const std::string& scanMatcherName) :
    mInputSetupTime(nullptr),
    mOptimizationTime(nullptr),
    mDiffTranslation(nullptr),
    mDiffRotation(nullptr),
    mWinSizeX(nullptr),
    mWinSizeY(nullptr),
    mWinSizeTheta(nullptr),
    mStepSizeX(nullptr),
    mStepSizeY(nullptr),
    mStepSizeTheta(nullptr),
    mNumOfIgnoredNodes(nullptr),
    mNumOfProcessedNodes(nullptr),
    mScoreValue(nullptr),
    mCostValue(nullptr),
    mNumOfScans(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the counter metrics */
    this->mNumOfIgnoredNodes = pMetricManager->AddCounter(
        scanMatcherName + ".NumOfIgnoredNodes");
    this->mNumOfProcessedNodes = pMetricManager->AddCounter(
        scanMatcherName + ".NumOfProcessedNodes");

    /* Register the distribution metrics */
    this->mInputSetupTime = pMetricManager->AddDistribution(
        scanMatcherName + ".InputSetupTime");
    this->mOptimizationTime = pMetricManager->AddDistribution(
        scanMatcherName + ".OptimizationTime");
    this->mDiffTranslation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffTranslation");
    this->mDiffRotation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffRotation");

    this->mWinSizeX = pMetricManager->AddDistribution(
        scanMatcherName + ".WinSizeX");
    this->mWinSizeY = pMetricManager->AddDistribution(
        scanMatcherName + ".WinSizeY");
    this->mWinSizeTheta = pMetricManager->AddDistribution(
        scanMatcherName + ".WinSizeTheta");
    this->mStepSizeX = pMetricManager->AddDistribution(
        scanMatcherName + ".StepSizeX");
    this->mStepSizeY = pMetricManager->AddDistribution(
        scanMatcherName + ".StepSizeY");
    this->mStepSizeTheta = pMetricManager->AddDistribution(
        scanMatcherName + ".StepSizeTheta");

    /* Register the value sequence metrics */
    this->mScoreValue = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".ScoreValue");
    this->mCostValue = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".CostValue");
    this->mNumOfScans = pMetricManager->AddValueSequenceInt(
        scanMatcherName + ".NumOfScans");
}

/* Constructor */
ScanMatcherBranchBound::ScanMatcherBranchBound(
    const std::string& scanMatcherName,
    const std::shared_ptr<ScorePixelAccurate>& scoreFunc,
    const CostFuncPtr& costFunc,
    const int nodeHeightMax,
    const double rangeX,
    const double rangeY,
    const double rangeTheta) :
    ScanMatcher(scanMatcherName),
    mScoreFunc(scoreFunc),
    mCostFunc(costFunc),
    mNodeHeightMax(nodeHeightMax),
    mRangeX(rangeX),
    mRangeY(rangeY),
    mRangeTheta(rangeTheta),
    mMetrics(scanMatcherName)
{
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherBranchBound::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Create the timer */
    Metric::Timer timer;

    /* Retrieve the query information */
    const auto& gridMap = queryInfo.mGridMap;
    const auto& scanData = queryInfo.mScanData;
    const RobotPose2D<double>& mapLocalInitialPose =
        queryInfo.mMapLocalInitialPose;

    /* Precompute the coarser grid maps */
    const auto precompMaps = this->ComputeCoarserMaps(gridMap);

    /* Update the metrics */
    this->mMetrics.mInputSetupTime->Observe(timer.ElapsedMicro());

    /* Optimize the robot pose by scan matching */
    return this->OptimizePose(gridMap, precompMaps, scanData,
                              mapLocalInitialPose, 0.0, 0.0);
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherBranchBound::OptimizePose(
    const GridMap& gridMap,
    const std::vector<ConstMap>& precompMaps,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalInitialPose,
    const double normalizedScoreThreshold,
    const double knownRateThreshold) const
{
    /* Create the timer */
    Metric::Timer timer;

    /* Find the best pose from the search window
     * that maximizes the matching score value */
    const RobotPose2D<double> mapLocalSensorPose =
        Compound(mapLocalInitialPose, scanData->RelativeSensorPose());

    /* Determine the search window step */
    double stepX;
    double stepY;
    double stepTheta;
    this->ComputeSearchStep(gridMap, scanData, stepX, stepY, stepTheta);

    /* Determine the search window */
    const int winX = static_cast<int>(
        std::ceil(0.5 * this->mRangeX / stepX));
    const int winY = static_cast<int>(
        std::ceil(0.5 * this->mRangeY / stepY));
    const int winTheta = static_cast<int>(
        std::ceil(0.5 * this->mRangeTheta / stepTheta));

    /* Setup the best score */
    double scoreMax = normalizedScoreThreshold;
    /* Setup the best pose */
    int bestX = 0;
    int bestY = 0;
    int bestTheta = 0;
    /* Setup the number of the ignored nodes and processed nodes */
    int numOfIgnoredNodes = 0;
    int numOfProcessedNodes = 0;

    /* Initialize a queue with nodes covering the entire search window */
    std::priority_queue<Node> nodeQueue;
    const int winSizeMax = 1 << this->mNodeHeightMax;

    /* Define a helper lambda function to append to the queue */
    auto appendNode = [&](const int x, const int y,
                          const int theta, const int height) {
        /* Compute the corresponding node pose */
        const RobotPose2D<double> nodePose {
            mapLocalSensorPose.mX + x * stepX,
            mapLocalSensorPose.mY + y * stepY,
            mapLocalSensorPose.mTheta + theta * stepTheta };
        /* Compute the node score */
        const ScoreFunction::Summary scoreSummary = this->mScoreFunc->Score(
            precompMaps.at(height), scanData, nodePose);
        /* Push the new node to the queue */
        if (scoreSummary.mNormalizedScore > scoreMax)
            nodeQueue.emplace(x, y, theta, height,
                              scoreSummary.mNormalizedScore,
                              scoreSummary.mKnownRate);
        /* Update the number of the ignored nodes
         * Number of the processed nodes `numOfProcessedNodes` is not updated
         * here since the node that we added above might not be processed */
        if (scoreSummary.mNormalizedScore <= scoreMax)
            numOfIgnoredNodes++;
    };

    /* Initialize a queue */
    for (int x = -winX; x <= winX; x += winSizeMax)
        for (int y = -winY; y <= winY; y += winSizeMax)
            for (int t = -winTheta; t <= winTheta; ++t)
                appendNode(x, y, t, this->mNodeHeightMax);

    /* Find the best solution that maximizes the score value
     * using Branch-and-Bound method */
    while (!nodeQueue.empty()) {
        /* Retrieve the node */
        const Node& currentNode = nodeQueue.top();

        /* Ignore the node if the score falls below the threshold */
        if (currentNode.mNormalizedScore <= scoreMax ||
            currentNode.mKnownRate <= knownRateThreshold) {
            /* Pop the current node from the stack */
            nodeQueue.pop();
            /* Update the number of the ignored nodes */
            numOfIgnoredNodes++;
            continue;
        }

        /* If the current node is a leaf node, update the solution */
        if (currentNode.IsLeafNode()) {
            /* Update the solution */
            bestX = currentNode.mX;
            bestY = currentNode.mY;
            bestTheta = currentNode.mTheta;
            scoreMax = currentNode.mNormalizedScore;

            /* Pop the current node from the stack */
            nodeQueue.pop();
            /* Update the number of the processed nodes */
            numOfProcessedNodes++;
        } else {
            /* Otherwise, split the current node into four new nodes */
            const int x = currentNode.mX;
            const int y = currentNode.mY;
            const int theta = currentNode.mTheta;
            const int height = currentNode.mHeight - 1;
            const int winSize = 1 << height;

            /* Pop the current node from the stack */
            nodeQueue.pop();
            /* Update the number of the processed nodes */
            numOfProcessedNodes++;

            /* Push the new nodes to the stack */
            appendNode(x, y, theta, height);
            appendNode(x + winSize, y, theta, height);
            appendNode(x, y + winSize, theta, height);
            appendNode(x + winSize, y + winSize, theta, height);
        }
    }

    /* The appropriate pose is found if the maximum score is larger than
     * (not larger than or equal to) the score threshold */
    const bool poseFound = scoreMax > normalizedScoreThreshold;

    /* Compute the best sensor pose */
    const RobotPose2D<double> bestSensorPose {
        mapLocalSensorPose.mX + stepX * bestX,
        mapLocalSensorPose.mY + stepY * bestY,
        mapLocalSensorPose.mTheta + stepTheta * bestTheta };
    /* Evaluate the cost value */
    const double costVal = this->mCostFunc->Cost(
        gridMap, scanData, bestSensorPose);
    /* Compute the normalized cost value */
    const double normalizedCost = costVal / scanData->NumOfScans();

    /* Compute the estimated robot pose in a map-local coordinate frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());
    /* Compute the pose covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Update the metrics */
    this->mMetrics.mOptimizationTime->Observe(timer.ElapsedMicro());
    this->mMetrics.mDiffTranslation->Observe(
        Distance(mapLocalInitialPose, estimatedPose));
    this->mMetrics.mDiffRotation->Observe(
        std::abs(mapLocalInitialPose.mTheta - estimatedPose.mTheta));
    this->mMetrics.mWinSizeX->Observe(winX);
    this->mMetrics.mWinSizeY->Observe(winY);
    this->mMetrics.mWinSizeTheta->Observe(winTheta);
    this->mMetrics.mStepSizeX->Observe(stepX);
    this->mMetrics.mStepSizeY->Observe(stepY);
    this->mMetrics.mStepSizeTheta->Observe(stepTheta);
    this->mMetrics.mNumOfIgnoredNodes->Increment(numOfIgnoredNodes);
    this->mMetrics.mNumOfProcessedNodes->Increment(numOfProcessedNodes);
    this->mMetrics.mScoreValue->Observe(scoreMax);
    this->mMetrics.mCostValue->Observe(normalizedCost);
    this->mMetrics.mNumOfScans->Observe(scanData->NumOfScans());

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated pose covariance matrix in a map-local frame */
    return ScanMatchingSummary {
        poseFound, normalizedCost, mapLocalInitialPose,
        estimatedPose, estimatedCovariance };
}

/* Precompute multiple coarser grid maps for scan matching */
std::vector<ConstMap> ScanMatcherBranchBound::ComputeCoarserMaps(
    const GridMap& gridMap) const
{
    /* Map with the tree height (key) and the coarser grid map (value) */
    std::vector<ConstMap> precompMaps;
    /* Create multiple coarser grid maps */
    PrecomputeGridMaps(gridMap, precompMaps, this->mNodeHeightMax);
    /* Return the grid map pyramid */
    return precompMaps;
}

/* Compute the search window step */
void ScanMatcherBranchBound::ComputeSearchStep(
    const GridMap& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    double& stepX,
    double& stepY,
    double& stepTheta) const
{
    /* Determine the search step */
    const double mapResolution = gridMap.Resolution();
    const auto maxRangeIt = std::max_element(
        scanData->Ranges().cbegin(), scanData->Ranges().cend());
    const double maxRange = *maxRangeIt;
    const double theta = mapResolution / maxRange;

    stepX = mapResolution;
    stepY = mapResolution;
    stepTheta = std::acos(1.0 - 0.5 * theta * theta);

    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
