
/* pose_graph_optimizer_factory.cpp */

#include "my_lidar_graph_slam/pose_graph_optimizer_factory.hpp"

#include "my_lidar_graph_slam/loss_function_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Create a Levenberg-Marquardt method based pose graph optimizer */
std::unique_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizerLM(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for Levenberg-Marquardt based pose graph optimizer */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    /* Convert linear solver type string to enum */
    using SolverType = Mapping::PoseGraphOptimizerLM::SolverType;
    const std::string solverTypeStr =
        config.get<std::string>("SolverType");
    const SolverType solverType =
        solverTypeStr == "SparseCholesky" ? SolverType::SparseCholesky :
        solverTypeStr == "ConjugateGradient" ? SolverType::ConjugateGradient :
        SolverType::SparseCholesky;

    const int numOfIterationsMax = config.get<int>("NumOfIterationsMax");
    const double errorTolerance = config.get<double>("ErrorTolerance");
    const double initialLambda = config.get<double>("InitialLambda");

    const std::string lossFuncType =
        config.get<std::string>("LossFunctionType");
    const std::string lossFuncConfigGroup =
        config.get<std::string>("LossFunctionConfigGroup");
    auto pLossFunction = CreateLossFunction(
        jsonSettings, lossFuncType, lossFuncConfigGroup);

    /* Construct pose graph optimizer object */
    auto pOptimizer = std::make_unique<Mapping::PoseGraphOptimizerLM>(
        solverType, numOfIterationsMax, errorTolerance, initialLambda,
        pLossFunction);

    return pOptimizer;
}

/* Create a g2o-based pose graph optimizer */
std::unique_ptr<Mapping::PoseGraphOptimizerG2O> CreatePoseGraphOptimizerG2O(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for g2o-based pose graph optimizer */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const int maxNumOfIterations =
        config.get<int>("MaxNumOfIterations");
    const double convergenceThreshold =
        config.get<double>("ConvergenceThreshold");

    /* Create g2o-based pose graph optimizer */
    auto pOptimizer = std::make_unique<Mapping::PoseGraphOptimizerG2O>(
        maxNumOfIterations, convergenceThreshold);

    return pOptimizer;
}

/* Create a pose graph optimizer */
std::unique_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizer(
    const pt::ptree& jsonSettings,
    const std::string& optimizerType,
    const std::string& configGroup)
{
    if (optimizerType == "LM")
        return CreatePoseGraphOptimizerLM(jsonSettings, configGroup);
    else if (optimizerType == "G2O")
        return CreatePoseGraphOptimizerG2O(jsonSettings, configGroup);

    return nullptr;
}

} /* namespace MyLidarGraphSlam */
