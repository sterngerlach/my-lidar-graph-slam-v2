
/* pose_graph_optimizer_g2o.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_G2O_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_G2O_HPP

#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"

#include <memory>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"

#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct PoseGraphOptimizerG2OMetrics
{
    /* Constructor */
    PoseGraphOptimizerG2OMetrics();
    /* Destructor */
    ~PoseGraphOptimizerG2OMetrics() = default;

    /* Total number of the iterations */
    Metric::ValueSequenceBase* mNumOfIterations;
    /* Initial error value */
    Metric::ValueSequenceBase* mInitialError;
    /* Final error value */
    Metric::ValueSequenceBase* mFinalError;
    /* Total number of the pose graph local map nodes */
    Metric::ValueSequenceBase* mNumOfLocalMapNodes;
    /* Total number of the pose graph scan nodes */
    Metric::ValueSequenceBase* mNumOfScanNodes;
    /* Total number of the pose graph edges */
    Metric::ValueSequenceBase* mNumOfEdges;
};

/*
 * PoseGraphOptimizerG2O class optimizes the pose graph using g2o
 */
class PoseGraphOptimizerG2O final : public PoseGraphOptimizer
{
public:
    /* Constructor */
    PoseGraphOptimizerG2O(const int maxNumOfIterations,
                          const double convergenceThreshold);

    /* Destructor */
    ~PoseGraphOptimizerG2O() = default;

    /* Copy constructor (deleted) */
    PoseGraphOptimizerG2O(const PoseGraphOptimizerG2O&) = delete;
    /* Copy assignment operator (deleted) */
    PoseGraphOptimizerG2O& operator=(const PoseGraphOptimizerG2O&) = delete;
    /* Move constructor */
    PoseGraphOptimizerG2O(PoseGraphOptimizerG2O&&) = default;
    /* Move assignment operator */
    PoseGraphOptimizerG2O& operator=(PoseGraphOptimizerG2O&&) = default;

    /* Optimize a pose graph using g2o */
    void Optimize(std::vector<Eigen::Vector3d>& localMapNodes,
                  std::vector<Eigen::Vector3d>& scanNodes,
                  const std::vector<EdgePose>& poseGraphEdges) override;

private:
    /* Create a sparse optimizer */
    void CreateOptimizer();

private:
    /* Maximum number of the optimization iterations */
    const int    mMaxNumOfIterations;
    /* Convergence threshold */
    const double mConvergenceThreshold;

    /* Sparse optimizer */
    std::unique_ptr<g2o::SparseOptimizer> mOptimizer;
    /* Metrics information */
    PoseGraphOptimizerG2OMetrics          mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_G2O_HPP */
