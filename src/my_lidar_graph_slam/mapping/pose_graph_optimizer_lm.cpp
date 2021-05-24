
/* pose_graph_optimizer_lm.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_lm.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
PoseGraphOptimizerLMMetrics::PoseGraphOptimizerLMMetrics() :
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
    this->mNumOfIterations = pMetricManager->AddValueSequenceInt(
        "PoseGraphOptimizerLM.NumOfIterations");
    this->mInitialError = pMetricManager->AddValueSequenceFloat(
        "PoseGraphOptimizerLM.InitialError");
    this->mFinalError = pMetricManager->AddValueSequenceFloat(
        "PoseGraphOptimizerLM.FinalError");
    this->mNumOfLocalMapNodes = pMetricManager->AddValueSequenceInt(
        "PoseGraphOptimizerLM.NumOfLocalMapNodes");
    this->mNumOfScanNodes = pMetricManager->AddValueSequenceInt(
        "PoseGraphOptimizerLM.NumOfScanNodes");
    this->mNumOfEdges = pMetricManager->AddValueSequenceInt(
        "PoseGraphOptimizerLM.NumOfEdges");
}

/* Optimize a pose graph using the combination of
 * linear solver and Levenberg-Marquardt method */
void PoseGraphOptimizerLM::Optimize(
    std::vector<Eigen::Vector3d>& localMapNodes,
    std::vector<Eigen::Vector3d>& scanNodes,
    const std::vector<EdgePose>& poseGraphEdges)
{
    double prevTotalError = std::numeric_limits<double>::max();
    double totalError = std::numeric_limits<double>::max();
    int numOfIterations = 0;

    /* Retrieve the number of nodes and edges
     * These remain constant during the optimization */
    const int numOfLocalMapNodes = static_cast<int>(localMapNodes.size());
    const int numOfScanNodes = static_cast<int>(scanNodes.size());
    const int numOfEdges = static_cast<int>(poseGraphEdges.size());

    /* Total number of the variables in the linear system */
    const int numOfVariables = 3 * (numOfLocalMapNodes + numOfScanNodes);
    /* Total number of the non-zero elements in the sparse matrix
     * Only lower triangular part is computed */
    const std::size_t numOfNonzeroElements =
        21 * numOfEdges + 3 + numOfVariables;

    /* Resize vectors and matrices */
    /* Resize the left-hand side sparse matrix of the linear system */
    this->mMatA.resize(numOfVariables, numOfVariables);
    /* Resize the right-hand side vector of the linear system */
    this->mVecB.resize(numOfVariables);
    /* Resize the result of the Sparse Cholesky factorization */
    this->mVecDelta.resize(numOfVariables);
    /* Resize and clear the vector of the sparse matrix non-zero elements */
    this->mMatATriplets.reserve(numOfNonzeroElements);
    this->mMatATriplets.clear();

    /* Compute the initial error */
    const double initialError = this->ComputeTotalError(
        localMapNodes, scanNodes, poseGraphEdges);

    while (true) {
        /* Perform one optimization step */
        this->OptimizeStep(localMapNodes, scanNodes, poseGraphEdges);
        /* Compute the total error */
        totalError = this->ComputeTotalError(
            localMapNodes, scanNodes, poseGraphEdges);

        /* Stop the graph optimization if the number of iteration steps
         * exceeded the maximum or the total error converged */
        if (++numOfIterations >= this->mNumOfIterationsMax ||
            std::fabs(prevTotalError - totalError) < this->mErrorTolerance)
            break;

        /* Update damping factor (lambda)
         * If error decreased, halve the damping factor
         * If error increased, double the damping factor */
        if (totalError < prevTotalError)
            this->mLambda *= 0.5;
        else
            this->mLambda *= 2.0;

        prevTotalError = totalError;
    }

    /* Update the metrics */
    this->mMetrics.mNumOfIterations->Observe(numOfIterations);
    this->mMetrics.mInitialError->Observe(initialError);
    this->mMetrics.mFinalError->Observe(totalError);
    this->mMetrics.mNumOfLocalMapNodes->Observe(numOfLocalMapNodes);
    this->mMetrics.mNumOfScanNodes->Observe(numOfScanNodes);
    this->mMetrics.mNumOfEdges->Observe(numOfEdges);
}

/* Perform one optimization step and return the total error */
void PoseGraphOptimizerLM::OptimizeStep(
    std::vector<Eigen::Vector3d>& localMapNodes,
    std::vector<Eigen::Vector3d>& scanNodes,
    const std::vector<EdgePose>& poseGraphEdges)
{
    const int numOfLocalMapNodes = static_cast<int>(localMapNodes.size());
    const int numOfScanNodes = static_cast<int>(scanNodes.size());
    const int numOfVariables = 3 * (numOfLocalMapNodes + numOfScanNodes);

    /* Initialize the triplets for sparse matrix in the first iteration */
    const bool isFirstIteration = this->mMatATriplets.empty();
    int tripletIdx = 3;

    if (isFirstIteration) {
        /* Add the sufficiently large value to the diagonals of the
         * first 3x3 block of H to fix the increments of the
         * first node pose to zero */
        for (int i = 0; i < 3; ++i)
            this->mMatATriplets.emplace_back(i, i, 1e9);

        /* Add the damping factor (lambda) to the diagonals of H */
        for (int i = 0; i < numOfVariables; ++i)
            this->mMatATriplets.emplace_back(i, i, this->mLambda);
    } else {
        /* Update the damping factor (lambda) */
        for (int i = 0; i < numOfVariables; ++i)
            this->mMatATriplets[tripletIdx++].Set(this->mLambda);
    }

    /* Clear the right-hand side vector containing residuals */
    this->mVecB.setZero();

    /* Initialize intermediate matrices */
    Eigen::Vector3d errorVec = Eigen::Vector3d::Zero();
    Eigen::Matrix3d startNodeJacobian = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d endNodeJacobian = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d trJsInfoJs = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d trJsInfoJe = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d trJeInfoJe = Eigen::Matrix3d::Zero();

    /* Setup the left-hand side sparse matrix H */
    for (const auto& edge : poseGraphEdges) {
        /* Retrieve the relative pose \bar{z}_{ij} from the edge */
        const Eigen::Vector3d& edgeRelPose = edge.mRelativePose;
        /* Retrieve the information matrix \Lambda_{ij} of the edge */
        const Eigen::Matrix3d& infoMat = edge.mInformationMat;

        /* Retrieve the poses of the start node and the end node */
        const Eigen::Vector3d& startNodePose =
            localMapNodes[edge.mLocalMapNodeIdx];
        const Eigen::Vector3d& endNodePose =
            scanNodes[edge.mScanNodeIdx];

        /* Compute the indices to the matrix elements */
        const int startNodeIdx = edge.mLocalMapNodeIdx;
        const int endNodeIdx = numOfLocalMapNodes + edge.mScanNodeIdx;

        /* Compute error function and Jacobian matrices */
        this->ComputeErrorAndJacobians(
            startNodePose, endNodePose, edgeRelPose,
            errorVec, startNodeJacobian, endNodeJacobian);

        /* Correct the information matrix using the weight function
         * based on the robust estimation (M-estimation)
         * to prevent outliers caused by wrong loop detections */
        const double errorWeight = edge.mIsLoopConstraint ?
            this->mLossFunction->Weight(
                errorVec.transpose() * infoMat * errorVec) : 1.0;

        /* Compute 4 non-zero block matrices denoted as 
         * J_i^T \Lambda_{ij} J_i, J_i^T \Lambda_{ij} J_j,
         * J_j^T \Lambda_{ij} J_i, and J_j^T \Lambda_{ij} J_j in the paper
         * Note that J_j^T \Lambda_{ij} J_i is the transpose of
         * J_i^T \Lambda_{ij} J_j and thus is not calculated */

        /* Compute the intermediate results, namely
         * J_i^T \Lambda_{ij} and J_j^T \Lambda_{ij} */
        const Eigen::Matrix3d trJsInfo =
            startNodeJacobian.transpose() * infoMat * errorWeight;
        const Eigen::Matrix3d trJeInfo =
            endNodeJacobian.transpose() * infoMat * errorWeight;

        /* Compute 3 non-zero block matrices
         * Only necessary elements are computed */
        trJsInfoJs.triangularView<Eigen::Lower>() =
            trJsInfo * startNodeJacobian;
        trJeInfoJe.triangularView<Eigen::Lower>() =
            trJeInfo * endNodeJacobian;
        trJsInfoJe = trJsInfo * endNodeJacobian;

        /* Update the non-zero elements of sparse matrix H
         * Only fill the elements in lower triangular part */
        const int startBaseIdx = 3 * startNodeIdx;
        const int endBaseIdx = 3 * endNodeIdx;

        if (isFirstIteration) {
            /* Fill the lower triangular part of J_i^T \Lambda_{ij} J_i and
             * J_j^T \Lambda_{ij} J_j to the lower triangular part of H */
            /* Then, fill the all elements of J_j^T \Lambda_{ij} J_i to the
             * lower triangular part of H */
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j <= i; ++j) {
                    /* Append the element of J_i^T \Lambda_{ij} J_i */
                    this->mMatATriplets.emplace_back(
                        startBaseIdx + i, startBaseIdx + j, trJsInfoJs(i, j));
                    /* Append the element of J_j^T \Lambda_{ij} J_j */
                    this->mMatATriplets.emplace_back(
                        endBaseIdx + i, endBaseIdx + j, trJeInfoJe(i, j));
                }

                for (int j = 0; j < 3; ++j) {
                    /* Append the element of J_j^T \Lambda_{ij} J_i,
                     * which is just the transpose of J_i^T \Lambda_{ij} J_j */
                    this->mMatATriplets.emplace_back(
                        endBaseIdx + i, startBaseIdx + j, trJsInfoJe(j, i));
                }
            }
        } else {
            /* Update the lower triangular part of J_i^T \Lambda_{ij} J_i and
             * J_j^T \Lambda_{ij} J_j to the lower triangular part of H */
            /* Then, update the all elements of J_j^T \Lambda_{ij} J_i to the
             * lower triangular part of H */
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j <= i; ++j) {
                    /* Update the element of J_i^T \Lambda_{ij} J_i */
                    this->mMatATriplets[tripletIdx++].Set(trJsInfoJs(i, j));
                    /* Update the element of J_j^T \Lambda_{ij} J_j */
                    this->mMatATriplets[tripletIdx++].Set(trJeInfoJe(i, j));
                }

                for (int j = 0; j < 3; ++j) {
                    /* Update the element of J_j^T \Lambda_{ij} J_i */
                    this->mMatATriplets[tripletIdx++].Set(trJsInfoJe(j, i));
                }
            }
        }

        /* Update the elements of vector J^T \Lambda e */
        this->mVecB.segment<3>(3 * startNodeIdx) -= trJsInfo * errorVec;
        this->mVecB.segment<3>(3 * endNodeIdx) -= trJeInfo * errorVec;
    }

    /* Create a sparse matrix H */
    this->mMatA.setFromTriplets(this->mMatATriplets.cbegin(),
                                this->mMatATriplets.cend());

    /* Solve the linear system */
    switch (this->mSolverType) {
        case SolverType::SparseCholesky: {
            /* Solve the linear system using sparse Cholesky factorization
             * and obtain the node pose increments */
            Eigen::SimplicialLDLT<
                Eigen::SparseMatrix<double>, Eigen::Lower> spCholSolver;
            /* Compute a sparse Cholesky decomposition of matrix H */
            spCholSolver.compute(this->mMatA);
            /* Solve the linear system for increment */
            this->mVecDelta = spCholSolver.solve(this->mVecB);
            break;
        }

        case SolverType::ConjugateGradient: {
            /* Solve the linear system using conjugate gradient method
             * and obtain the node pose increments */
            Eigen::ConjugateGradient<
                Eigen::SparseMatrix<double>, Eigen::Lower> cgSolver;
            /* Initialize a conjugate gradient linear solver with matrix A */
            cgSolver.compute(this->mMatA);
            /* Solve the linear system for increment */
            this->mVecDelta = cgSolver.solve(this->mVecB);
            break;
        }

        default: {
            /* Unknown solver type and program is aborted */
            assert(false && "Unknown linear solver type for optimization");
            break;
        }
    }

    /* Update the local map poses stored in pose graph */
    for (int i = 0; i < numOfLocalMapNodes; ++i)
        localMapNodes[i] += this->mVecDelta.segment<3>(3 * i);

    /* Update the scan node poses stored in the pose graph */
    for (int i = 0; i < numOfScanNodes; ++i)
        scanNodes[i] += this->mVecDelta.segment<3>(
            3 * (numOfLocalMapNodes + i));

    return;
}

/* Compute Jacobian matrices of the error function with respect to the
 * starting pose and ending pose of the pose graph edge */
void PoseGraphOptimizerLM::ComputeErrorJacobians(
    const Eigen::Vector3d& startNodePose,
    const Eigen::Vector3d& endNodePose,
    Eigen::Matrix3d& startNodeErrorJacobian,
    Eigen::Matrix3d& endNodeErrorJacobian) const
{
    /* Error function e_{ij} is defined as
     * e_{ij} = h(c_i, c_j) - \bar{z}_{ij} where
     * \bar{z}_{ij} is a measured offset between node c_i and c_j,
     * c_i = [x_i, y_i, \theta_i]^T and c_j = [x_j, y_j, \theta_j]^T are
     * the poses of the nodes to be optimized, and h(c_i, c_j) is a
     * function that computes the relative pose between c_i and c_j in
     * the frame of c_i */

    /* Function h(c_i, c_j) is written as follows:
     * h(c_i, c_j) = [ R_i^T (t_j - t_i),
     *                 \theta_j - \theta_i ]
     *             = [  cos \theta_i (x_j - x_i) + \sin \theta_i (y_j - y_i)
     *                 -sin \theta_i (x_j - x_i) + \cos \theta_i (y_j - y_i)
     *                  \theta_j - \theta_i ] */

    const double diffX = endNodePose[0] - startNodePose[0];
    const double diffY = endNodePose[1] - startNodePose[1];
    const double sinTheta = std::sin(startNodePose[2]);
    const double cosTheta = std::cos(startNodePose[2]);

    const double errorThetaPartialX = -sinTheta * diffX + cosTheta * diffY;
    const double errorThetaPartialY = -cosTheta * diffX - sinTheta * diffY;

    /* Compute a Jacobian matrix of the error function with respect to the
     * pose in the starting node */
    /* J_i = \frac{\partial e_{ij}}{\partial c_i}
     *     = [ -R_i^T, \frac{\partial R_i^T}{\partial \theta_i} (t_j - t_i)
     *          0,     -1 ]
     *     = [ -cos \theta_i, -sin \theta_i,  a
     *          sin \theta_i, -cos \theta_i,  b
     *          0,             0,            -1 ]
     * a = -sin \theta_i (x_j - x_i) + \cos \theta_i (y_j - y_i)
     * b = -cos \theta_i (x_j - x_i) - \sin \theta_i (y_j - y_i) */
    startNodeErrorJacobian << -cosTheta, -sinTheta, errorThetaPartialX,
                               sinTheta, -cosTheta, errorThetaPartialY,
                                    0.0,       0.0,               -1.0;

    /* Compute a Jacobian matrix of the error function with respect to the
     * pose in the end node */
    /* J_j = \frac{\partial e_{ij}}{\partial c_j}
     *     = [ R_i^T, 0
     *         0,     1 ]
     *     = [  cos \theta_i, sin \theta_i, 0
     *         -sin \theta_i, cos \theta_i, 0
     *          0,            0,            1 ] */
    endNodeErrorJacobian <<  cosTheta, sinTheta, 0.0,
                            -sinTheta, cosTheta, 0.0,
                                  0.0,      0.0, 1.0;

    return;
}

/* Compute error function */
void PoseGraphOptimizerLM::ComputeErrorFunction(
    const Eigen::Vector3d& startNodePose,
    const Eigen::Vector3d& endNodePose,
    const Eigen::Vector3d& edgeRelPose,
    Eigen::Vector3d& errorVec) const
{
    /* Compute the relative pose h(c_i, c_j) */
    const Eigen::Vector3d nodeRelPose =
        InverseCompound(startNodePose, endNodePose);
    
    /* Compute an error function e_{ij} = h(c_i, c_j) - \bar{z}_{ij} */
    errorVec << nodeRelPose[0] - edgeRelPose[0],
                nodeRelPose[1] - edgeRelPose[1],
                NormalizeAngle(nodeRelPose[2] - edgeRelPose[2]);

    return;
}

/* Compute error function and Jacobian matrices at one time */
void PoseGraphOptimizerLM::ComputeErrorAndJacobians(
    const Eigen::Vector3d& startNodePose,
    const Eigen::Vector3d& endNodePose,
    const Eigen::Vector3d& edgeRelPose,
    Eigen::Vector3d& errorVec,
    Eigen::Matrix3d& startNodeErrorJacobian,
    Eigen::Matrix3d& endNodeErrorJacobian) const
{
    const double sinTheta = std::sin(startNodePose[2]);
    const double cosTheta = std::cos(startNodePose[2]);

    const Eigen::Vector3d diffPose = endNodePose - startNodePose;

    const double x =  cosTheta * diffPose[0] + sinTheta * diffPose[1];
    const double y = -sinTheta * diffPose[0] + cosTheta * diffPose[1];

    /* Compute the relative pose h(c_i, c_j) */
    errorVec << x - edgeRelPose[0],
                y - edgeRelPose[1],
                NormalizeAngle(diffPose[2] - edgeRelPose[2]);

    /* Compute a Jacobian matrix of the error function with respect to the
     * pose in the starting node */
    startNodeErrorJacobian << -cosTheta, -sinTheta,    y,
                               sinTheta, -cosTheta,   -x,
                                    0.0,       0.0, -1.0;

    /* Compute a Jacobian matrix of the error function with respect to the
     * pose in the end node */
    endNodeErrorJacobian <<  cosTheta, sinTheta, 0.0,
                            -sinTheta, cosTheta, 0.0,
                                  0.0,      0.0, 1.0;

    return;
}

/* Compute total error */
double PoseGraphOptimizerLM::ComputeTotalError(
    const std::vector<Eigen::Vector3d>& localMapNodes,
    const std::vector<Eigen::Vector3d>& scanNodes,
    const std::vector<EdgePose>& poseGraphEdges) const
{
    double totalError = 0.0;

    /* Compute error function for each edge */
    for (const auto& edge : poseGraphEdges) {
        /* Retrieve the relative pose \bar{z}_{ij} from the edge */
        const Eigen::Vector3d& edgeRelPose = edge.mRelativePose;
        /* Retrieve the information matrix \Lambda_{ij} of the edge */
        const Eigen::Matrix3d& infoMat = edge.mInformationMat;

        /* Retrieve the poses of the start node and the end node */
        const Eigen::Vector3d& startNodePose =
            localMapNodes[edge.mLocalMapNodeIdx];
        const Eigen::Vector3d& endNodePose =
            scanNodes[edge.mScanNodeIdx];

        /* Compute the residual */
        Eigen::Vector3d errorVec;
        this->ComputeErrorFunction(startNodePose, endNodePose,
                                   edgeRelPose, errorVec);
        /* Compute the error value */
        const double errorVal = errorVec.transpose() * infoMat * errorVec;
        /* Apply the robust loss function */
        const double correctedError = this->mLossFunction->Loss(errorVal);

        /* Compute the error value */
        totalError += correctedError;
    }

    return totalError;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
