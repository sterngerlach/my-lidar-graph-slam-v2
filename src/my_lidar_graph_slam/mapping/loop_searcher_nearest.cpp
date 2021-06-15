
/* loop_searcher_nearest.cpp */

#include <cassert>
#include <numeric>

#include "my_lidar_graph_slam/mapping/loop_searcher_nearest.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopSearcherNearestMetrics::LoopSearcherNearestMetrics() :
    mAccumTravelDist(nullptr),
    mNodeDist(nullptr),
    mNumOfCandidateNodes(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the value sequence metrics */
    this->mAccumTravelDist = pMetricManager->AddValueSequence<float>(
        "LoopSearcherNearest.AccumTravelDist");
    this->mNodeDist = pMetricManager->AddValueSequence<float>(
        "LoopSearcherNearest.NodeDist");
    this->mNumOfCandidateNodes = pMetricManager->AddValueSequence<int>(
        "LoopSearcherNearest.NumOfCandidateNodes");
}

/*
 * CandidateDistance struct stores a squared distance `mDistSq` between
 * a query scan node with an Id `mQueryScanNodeId` and a reference scan node
 * with an Id `mRefScanNodeId`, which resides in a reference local map with
 * an Id `mRefLocalMapNodeId`
 */
struct CandidateDistance
{
    /* Constructor */
    CandidateDistance(const LocalMapId refLocalMapNodeId,
                      const NodeId refScanNodeId,
                      const NodeId queryScanNodeId,
                      const double distSq) :
        mRefLocalMapNodeId(refLocalMapNodeId),
        mRefScanNodeId(refScanNodeId),
        mQueryScanNodeId(queryScanNodeId),
        mDistSq(distSq) { }

    /* Id of the reference local map node */
    LocalMapId mRefLocalMapNodeId;
    /* Id of the reference scan node */
    NodeId     mRefScanNodeId;
    /* Id of the query scan node */
    NodeId     mQueryScanNodeId;
    /* Squared distance between the reference node and the query node */
    double     mDistSq;
};

/* Find a local map and a scan node used for loop detection */
LoopCandidateVector LoopSearcherNearest::Search(
    const LoopSearchHint& searchHint)
{
    /* Retrieve the pose graph nodes */
    const auto& scanNodes = searchHint.mScanNodes;
    const auto& localMapNodes = searchHint.mLocalMapNodes;
    /* Retrieve the accumulated travel distance of the robot */
    const double accumTravelDist = searchHint.mAccumTravelDist;
    /* Compute the squared distance threshold */
    const double nodeDistThresholdSq = std::pow(this->mNodeDistThreshold, 2.0);

    /* Store the distance between a query scan node inside the latest local map
     * and a reference scan node inside the old local map */
    std::vector<CandidateDistance> loopCandidateDistances;

    /* Check the last finished local map Id */
    Assert(localMapNodes.IdMax() == searchHint.mLastFinishedMapId);
    /* Retrieve query scan nodes inside the latest local map */
    const auto& queryMap = localMapNodes.at(searchHint.mLastFinishedMapId);
    const auto& queryNodeRange = scanNodes.RangeFromId(
        queryMap.mScanNodeIdMin, queryMap.mScanNodeIdMax);

    /* Retrieve the range of reference local maps */
    const auto referenceMapRange = localMapNodes.RangeFromIterator(
        localMapNodes.begin(), std::prev(localMapNodes.end()));

    double nodeTravelDist = 0.0;
    bool isFirstNode = true;
    RobotPose2D<double> prevPose = RobotPose2D<double>::Zero;

    for (const auto& [refMapId, refMapNode] : referenceMapRange) {
        /* Make sure that this reference local map is finished */
        Assert(refMapNode.mFinished);

        /* Retrieve two iterators pointing the scan nodes in this local map */
        const auto firstIt = scanNodes.find(refMapNode.mScanNodeIdMin);
        const auto lastIt = scanNodes.find(refMapNode.mScanNodeIdMax);
        /* Make sure that the iterators are valid */
        Assert(firstIt != scanNodes.end());
        Assert(lastIt != scanNodes.end());
        /* Retrieve the reference scan nodes inside this local map */
        const auto refScanRange = scanNodes.RangeFromIterator(
            firstIt, std::next(lastIt));

        for (const auto& [refId, refNode] : refScanRange) {
            /* Retrieve the global pose of this reference scan node */
            const RobotPose2D<double>& refPose = refNode.mGlobalPose;

            /* Compute the accumulated travel distance */
            nodeTravelDist += isFirstNode ? 0.0 : Distance(prevPose, refPose);
            prevPose = refPose;
            isFirstNode = false;

            /* Stop the iteration if the travel distance difference falls below
             * the specified threshold */
            if (accumTravelDist - nodeTravelDist < this->mTravelDistThreshold)
                goto Done;

            /* Compute the distance between this reference scan node and
             * the query scan nodes inside the latest local map */
            for (const auto& [queryId, queryNode] : queryNodeRange) {
                /* Retrieve the global pose of this query scan node */
                const RobotPose2D<double>& queryPose = queryNode.mGlobalPose;
                /* Compute the squared distance between two nodes */
                const double nodeDist = SquaredDistance(refPose, queryPose);
                /* Insert the distance if it satisfies the threshold */
                if (nodeDist < nodeDistThresholdSq)
                    loopCandidateDistances.emplace_back(
                        refMapId, refId, queryId, nodeDist);
            }
        }
    }

Done:
    /* Return an empty collection of candidates if not found */
    if (loopCandidateDistances.empty()) {
        /* Update the metrics */
        this->mMetrics.mAccumTravelDist->Observe(accumTravelDist);
        this->mMetrics.mNumOfCandidateNodes->Observe(0);
        return LoopCandidateVector { };
    }

    /* Sort the computed distances */
    const std::size_t numOfCandidates = std::min(
        static_cast<std::size_t>(this->mNumOfCandidateNodes),
        loopCandidateDistances.size());
    std::nth_element(
        loopCandidateDistances.begin(),
        loopCandidateDistances.begin() + numOfCandidates,
        loopCandidateDistances.end(),
        [](const CandidateDistance& lhs, const CandidateDistance& rhs) {
            return lhs.mDistSq < rhs.mDistSq; });

    /* Compute the loop candidates */
    LoopCandidateVector loopCandidates;
    loopCandidates.reserve(numOfCandidates);

    for (std::size_t i = 0; i < numOfCandidates; ++i)
        loopCandidates.emplace_back(
            loopCandidateDistances[i].mQueryScanNodeId,
            loopCandidateDistances[i].mRefScanNodeId,
            loopCandidateDistances[i].mRefLocalMapNodeId);

    /* Update the metrics */
    this->mMetrics.mAccumTravelDist->Observe(accumTravelDist);
    this->mMetrics.mNumOfCandidateNodes->Observe(numOfCandidates);

    for (std::size_t i = 0; i < numOfCandidates; ++i)
        this->mMetrics.mNodeDist->Observe(loopCandidateDistances[i].mDistSq);

    return loopCandidates;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
