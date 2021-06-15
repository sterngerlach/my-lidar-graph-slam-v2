
/* pose_graph_optimizer_lm.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP

#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/pose_eigen.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/robust_loss_function.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct PoseGraphOptimizerLMMetrics
{
    /* Constructor */
    PoseGraphOptimizerLMMetrics();
    /* Destructor */
    ~PoseGraphOptimizerLMMetrics() = default;

    /* Total number of the iterations */
    Metric::ValueSequenceBase<int>*   mNumOfIterations;
    /* Initial error value */
    Metric::ValueSequenceBase<float>* mInitialError;
    /* Final error value */
    Metric::ValueSequenceBase<float>* mFinalError;
    /* Total number of the pose graph local map nodes */
    Metric::ValueSequenceBase<int>*   mNumOfLocalMapNodes;
    /* Total number of the pose graph scan nodes */
    Metric::ValueSequenceBase<int>*   mNumOfScanNodes;
    /* Total number of the pose graph edges */
    Metric::ValueSequenceBase<int>*   mNumOfEdges;
};

/*
 * MutableTriplet struct derives from Eigen::Triplet<double> and enables to
 * overwrite the value of the element in sparse matrix
 */
class MutableTriplet final : public Eigen::Triplet<double>
{
public:
    /* Type definitions for convenience */
    using Scalar = double;
    using StorageIndex = typename Eigen::SparseMatrix<Scalar>::StorageIndex;

    /* Default constructor */
    MutableTriplet() : Triplet() { }
    /* Constructor with index and element */
    MutableTriplet(const StorageIndex& i,
                   const StorageIndex& j,
                   const Scalar& v = Scalar(0)) :
        Triplet(i, j, v) { }
    /* Destructor */
    ~MutableTriplet() = default;

    /* Set the value of the element */
    void Set(const Scalar& value) { this->m_value = value; }
};

/*
 * PoseGraphOptimizerLM class optimizes the pose graph using the
 * combination of linear solver and Levenberg-Marquardt method
 * which is also called Sparse Pose Adjustment (SPA) as proposed in the
 * following paper:
 * K. Konolige, G. Grisetti, R. Kuemmerle, W. Burgard, B. Limketkai, and
 * R. Vincent. "Efficient Sparse Pose Adjustment for 2D Mapping," in the
 * Proceedings of the IEEE/RSJ International Conference on Intelligent Robots
 * and Systems (IROS), 2010.
 */
class PoseGraphOptimizerLM final : public PoseGraphOptimizer
{
public:
    /*
     * SolverType enum specifies the linear solver used in this class
     */
    enum class SolverType
    {
        SparseCholesky,
        ConjugateGradient,
    };

public:
    /* Constructor */
    PoseGraphOptimizerLM(const SolverType solverType,
                         const int numOfIterationsMax,
                         const double errorTolerance,
                         const double initialLambda,
                         const LossFunctionPtr& lossFunction) :
        mSolverType(solverType),
        mNumOfIterationsMax(numOfIterationsMax),
        mErrorTolerance(errorTolerance),
        mLambda(initialLambda),
        mLossFunction(lossFunction),
        mMetrics() { }
    /* Destructor */
    ~PoseGraphOptimizerLM() = default;

    /* Optimize a pose graph using the combination of
     * linear solver and Levenberg-Marquardt method */
    void Optimize(
        std::vector<Eigen::Vector3d>& localMapNodes,
        std::vector<Eigen::Vector3d>& scanNodes,
        const std::vector<EdgePose>& poseGraphEdges) override;

private:
    /* Perform one optimization step and return the total error */
    void OptimizeStep(
        std::vector<Eigen::Vector3d>& localMapNodes,
        std::vector<Eigen::Vector3d>& scanNodes,
        const std::vector<EdgePose>& poseGraphEdges);

    /* Compute Jacobian matrices of the error function with respect to the
     * starting pose and ending pose of the pose graph edge */
    void ComputeErrorJacobians(
        const Eigen::Vector3d& startNodePose,
        const Eigen::Vector3d& endNodePose,
        Eigen::Matrix3d& startNodeErrorJacobian,
        Eigen::Matrix3d& endNodeErrorJacobian) const;

    /* Compute error function */
    void ComputeErrorFunction(
        const Eigen::Vector3d& startNodePose,
        const Eigen::Vector3d& endNodePose,
        const Eigen::Vector3d& edgeRelPose,
        Eigen::Vector3d& errorVec) const;

    /* Compute error function and Jacobian matrices at one time */
    void ComputeErrorAndJacobians(
        const Eigen::Vector3d& startNodePose,
        const Eigen::Vector3d& endNodePose,
        const Eigen::Vector3d& edgeRelPose,
        Eigen::Vector3d& errorVec,
        Eigen::Matrix3d& startNodeErrorJacobian,
        Eigen::Matrix3d& endNodeErrorJacobian) const;

    /* Compute total error */
    double ComputeTotalError(
        const std::vector<Eigen::Vector3d>& localMapNodes,
        const std::vector<Eigen::Vector3d>& scanNodes,
        const std::vector<EdgePose>& poseGraphEdges) const;

private:
    /* Linear solver type */
    const SolverType            mSolverType;
    /* Maximum number of the optimization iterations */
    const int                   mNumOfIterationsMax;
    /* Error tolerance to check the convergence */
    const double                mErrorTolerance;
    /* Damping factor used in Levenberg-Marquardt method
     * The method is almost the same as Gauss-Newton method when small,
     * and is gradient descent method when large */
    double                      mLambda;
    /* Robust loss function to correct (weight) information matrices */
    LossFunctionPtr             mLossFunction;

    /* Left-hand side sparse matrix of the linear system */
    Eigen::SparseMatrix<double> mMatA;
    /* Right-hand side vector of the linear system */
    Eigen::VectorXd             mVecB;
    /* Result of the Sparse Cholesky factorization */
    Eigen::VectorXd             mVecDelta;
    /* Vector of the sparse matrix non-zero elements (triplets) */
    std::vector<MutableTriplet> mMatATriplets;
    /* Metrics information */
    PoseGraphOptimizerLMMetrics mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP */
