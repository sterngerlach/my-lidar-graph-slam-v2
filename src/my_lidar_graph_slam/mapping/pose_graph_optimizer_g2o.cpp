
/* pose_graph_optimizer_g2o.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_g2o.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
PoseGraphOptimizerG2OMetrics::PoseGraphOptimizerG2OMetrics() :
    mNumOfIterations(nullptr),
    mInitialError(nullptr),
    mFinalError(nullptr),
    mNumOfLocalMapNodes(nullptr),
    mNumOfScanNodes(nullptr),
    mNumOfEdges(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the value sequence metrics */
    this->mNumOfIterations = pMetricManager->AddValueSequence<int>(
        "PoseGraphOptimizerG2O.NumOfIterations");
    this->mInitialError = pMetricManager->AddValueSequence<float>(
        "PoseGraphOptimizerG2O.InitialError");
    this->mFinalError = pMetricManager->AddValueSequence<float>(
        "PoseGraphOptimizerG2O.FinalError");
    this->mNumOfLocalMapNodes = pMetricManager->AddValueSequence<int>(
        "PoseGraphOptimizerG2O.NumOfLocalMapNodes");
    this->mNumOfScanNodes = pMetricManager->AddValueSequence<int>(
        "PoseGraphOptimizerG2O.NumOfScanNodes");
    this->mNumOfEdges = pMetricManager->AddValueSequence<int>(
        "PoseGraphOptimizerG2O.NumOfEdges");
}

/* Constructor */
PoseGraphOptimizerG2O::PoseGraphOptimizerG2O(
    const int maxNumOfIterations,
    const double convergenceThreshold) :
    mMaxNumOfIterations(maxNumOfIterations),
    mConvergenceThreshold(convergenceThreshold),
    mOptimizer(),
    mMetrics()
{
    /* Create a sparse optimizer */
    this->CreateOptimizer();
}

/* Optimize a pose graph using g2o */
void PoseGraphOptimizerG2O::Optimize(
    std::vector<Eigen::Vector3d>& localMapNodes,
    std::vector<Eigen::Vector3d>& scanNodes,
    const std::vector<EdgePose>& poseGraphEdges)
{
    /* Retrieve the number of nodes and edges */
    const int numOfLocalMapNodes = static_cast<int>(localMapNodes.size());
    const int numOfScanNodes = static_cast<int>(scanNodes.size());
    const int numOfEdges = static_cast<int>(poseGraphEdges.size());

    /* Clear the optimizer */
    this->mOptimizer->clear();
    this->mOptimizer->clearParameters();

    /* Append the local map nodes to the optimizer */
    for (int i = 0; i < numOfLocalMapNodes; ++i) {
        g2o::VertexSE2* pVertex = new g2o::VertexSE2();
        pVertex->setId(i);
        pVertex->setEstimate(g2o::SE2(localMapNodes[i]));
        this->mOptimizer->addVertex(pVertex);
    }

    /* Append the scan nodes to the optimizer */
    for (int i = 0; i < numOfScanNodes; ++i) {
        g2o::VertexSE2* pVertex = new g2o::VertexSE2();
        pVertex->setId(numOfLocalMapNodes + i);
        pVertex->setEstimate(g2o::SE2(scanNodes[i]));
        this->mOptimizer->addVertex(pVertex);
    }

    /* Append the edges to the optimizer */
    for (int i = 0; i < numOfEdges; ++i) {
        const auto& edge = poseGraphEdges[i];
        g2o::EdgeSE2* pEdge = new g2o::EdgeSE2();
        pEdge->vertices()[0] = this->mOptimizer->vertex(
            edge.mLocalMapNodeIdx);
        pEdge->vertices()[1] = this->mOptimizer->vertex(
            numOfLocalMapNodes + edge.mScanNodeIdx);
        pEdge->setMeasurement(g2o::SE2(edge.mRelativePose));
        pEdge->setInformation(edge.mInformationMat);
        this->mOptimizer->addEdge(pEdge);
    }

    /* Fix the first local map node */
    auto* pFirstNode = dynamic_cast<g2o::VertexSE2*>(
        this->mOptimizer->vertex(0));
    pFirstNode->setFixed(true);

    /* Initialize the optimizer */
    this->mOptimizer->initializeOptimization();
    /* Compute the initial error */
    const double initialError = this->mOptimizer->chi2();
    /* Optimize the pose graph */
    const int numOfIterations = this->mOptimizer->optimize(
        this->mMaxNumOfIterations);
    /* Compute the final error */
    const double finalError = this->mOptimizer->chi2();

    /* Update the local map poses stored in the pose graph */
    for (int i = 0; i < numOfLocalMapNodes; ++i) {
        const auto* pVertex = dynamic_cast<g2o::VertexSE2*>(
            this->mOptimizer->vertex(i));
        const g2o::SE2& poseEstimate = pVertex->estimate();
        localMapNodes[i][0] = poseEstimate.translation()[0];
        localMapNodes[i][1] = poseEstimate.translation()[1];
        localMapNodes[i][2] = poseEstimate.rotation().angle();
    }

    /* Update the scan node poses stored in the pose graph */
    for (int i = 0; i < numOfScanNodes; ++i) {
        const auto* pVertex = dynamic_cast<g2o::VertexSE2*>(
            this->mOptimizer->vertex(numOfLocalMapNodes + i));
        const g2o::SE2& poseEstimate = pVertex->estimate();
        scanNodes[i][0] = poseEstimate.translation()[0];
        scanNodes[i][1] = poseEstimate.translation()[1];
        scanNodes[i][2] = poseEstimate.rotation().angle();
    }

    /* Update the metrics */
    this->mMetrics.mNumOfIterations->Observe(numOfIterations);
    this->mMetrics.mInitialError->Observe(initialError);
    this->mMetrics.mFinalError->Observe(finalError);
    this->mMetrics.mNumOfLocalMapNodes->Observe(numOfLocalMapNodes);
    this->mMetrics.mNumOfScanNodes->Observe(numOfScanNodes);
    this->mMetrics.mNumOfEdges->Observe(numOfEdges);

    return;
}

/* Create a sparse optimizer */
void PoseGraphOptimizerG2O::CreateOptimizer()
{
    /* Type declarations for convenience */
    /* g2o::BlockSolverX or g2o::BlockSolver_3_2 can be used
     * g2o::BlockSolverX seems to be faster than g2o::BlockSolver_3_2 */
    using BlockSolver = g2o::BlockSolverX;
    /* g2o::LinearSolverEigen, g2o::LinearSolverCholmod or
     * g2o::LinearSolverCSparse can be used
     * g2o::LinearSolverCholmod and g2o::LinearSolverCSparse seem to show
     * better performance than g2o::LinearSolverEigen */
    using LinearSolver = g2o::LinearSolverCholmod<BlockSolver::PoseMatrixType>;

    /* Create a Cholmod-based linear solver */
    auto pLinearSolver = std::make_unique<LinearSolver>();
    pLinearSolver->setBlockOrdering(false);
    pLinearSolver->setWriteDebug(false);

    /* Create a block solver */
    auto pBlockSolver = std::make_unique<BlockSolver>(
        std::move(pLinearSolver));
    pBlockSolver->setWriteDebug(false);

    /* Create a Gauss-Newton optimizer */
    auto pGaussNewton = new g2o::OptimizationAlgorithmGaussNewton(
        std::move(pBlockSolver));
    pGaussNewton->setWriteDebug(false);

    /* Create a sparse optimizer */
    auto pOptimizer = std::make_unique<g2o::SparseOptimizer>();
    pOptimizer->setAlgorithm(pGaussNewton);
    pOptimizer->setVerbose(false);

    /* Set the convergence criterion */
    auto* pTerminateAction = new g2o::SparseOptimizerTerminateAction();
    pTerminateAction->setMaxIterations(this->mMaxNumOfIterations);
    pTerminateAction->setGainThreshold(this->mConvergenceThreshold);
    pOptimizer->addPostIterationAction(pTerminateAction);

    /* Set the sparse optimizer */
    this->mOptimizer = std::move(pOptimizer);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
