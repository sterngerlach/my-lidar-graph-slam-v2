
/* slam_launcher.cpp */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <vector>

#ifdef __GNUC__
#if (__GNUC__ >= 6) && (__GNUC__ < 8)
#include <experimental/filesystem>
#elif (__GNUC__ >= 8)
#include <filesystem>
#endif
#endif

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "my_lidar_graph_slam/slam_module_factory.hpp"

#include "my_lidar_graph_slam/hw/bitstream_loader.hpp"
#include "my_lidar_graph_slam/hw/cma_manager.hpp"
#include "my_lidar_graph_slam/io/gnuplot_helper.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/io/carmen/carmen_reader.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

using namespace MyLidarGraphSlam;

/* Declare namespaces for convenience */
namespace pt = boost::property_tree;

#ifdef __GNUC__
#if (__GNUC__ >= 6) && (__GNUC__ < 8)
namespace fs = std::experimental::filesystem;
#elif (__GNUC__ >= 8)
namespace fs = std::filesystem;
#endif
#endif

/* Load Carmen log data */
void LoadCarmenLog(const fs::path& logFilePath,
                   std::vector<Sensor::SensorDataPtr>& logData)
{
    /* Open the carmen log file */
    std::ifstream logFile { logFilePath };

    if (!logFile) {
        std::cerr << "Failed to open log file: " << logFilePath << std::endl;
        return;
    }

    /* Load the carmen log file */
    IO::Carmen::CarmenLogReader logReader;
    logReader.Load(logFile, logData);
    logFile.close();
}

/* LauncherSettings struct stores configurations for SLAM launcher */
struct LauncherSettings
{
    bool        mGuiEnabled;
    int         mDrawFrameInterval;
    bool        mWaitForKey;
};

/* Load the bitstream file to enable the hardware acceleration */
bool LoadBitstream(const pt::ptree& jsonSettings)
{
    /* Read settings for the hardware acceleration */
    const bool enableHardwareAcceleration =
        jsonSettings.get<bool>("Hardware.EnableHardwareAcceleration");
    const std::string bitstreamFileName =
        jsonSettings.get<std::string>("Hardware.BitstreamFileName");

    if (!enableHardwareAcceleration)
        return true;

    /* Load the shared object library for the CMA memory management */
    auto* const pCmaManager = Hardware::CMAMemoryManager::Instance();

    if (!pCmaManager->Load())
        return false;

    /* Load the specified bitstream file */
    Hardware::BitstreamLoader bitstreamLoader;

    if (!bitstreamLoader.Load(bitstreamFileName))
        return false;

    return true;
}

/* Load settings from JSON format configuration file */
bool LoadSettings(const fs::path& settingsFilePath,
                  Mapping::LidarGraphSlamPtr& pLidarGraphSlam,
                  LauncherSettings& launcherSettings)
{
    /* Load settings from JSON configuration file */
    pt::ptree jsonSettings;
    pt::read_json(settingsFilePath, jsonSettings);

    /* Read settings for gnuplot GUI */
    launcherSettings.mGuiEnabled =
        jsonSettings.get<bool>("Launcher.GuiEnabled");
    launcherSettings.mDrawFrameInterval =
        jsonSettings.get<int>("Launcher.DrawFrameInterval");

    /* Read settings for SLAM launcher */
    launcherSettings.mWaitForKey =
        jsonSettings.get<bool>("Launcher.WaitForKey");

    /* Load the bitstream file to enable the hardware acceleration
     * before setting up the scan matcher and the loop detector */
    if (!LoadBitstream(jsonSettings))
        return false;

    /* Construct LiDAR Graph-Based SLAM */
    pLidarGraphSlam = CreateLidarGraphSlam(jsonSettings);

    return true;
}

/* Draw pose graph on Gnuplot window */
void DrawPoseGraph(const Mapping::LidarGraphSlamPtr& pLidarGraphSlam,
                   const std::unique_ptr<IO::GnuplotHelper>& pGnuplotHelper)
{
    /* Get the pose graph information */
    Mapping::IdMap<Mapping::LocalMapId, Mapping::LocalMapNode> localMapNodes;
    Mapping::IdMap<Mapping::NodeId, Mapping::ScanNodeData> scanNodes;
    std::vector<Mapping::EdgeData> poseGraphEdges;
    pLidarGraphSlam->GetPoseGraph(localMapNodes, scanNodes, poseGraphEdges);

    /* Draw the current pose graph if necessary */
    pGnuplotHelper->DrawPoseGraph(localMapNodes, scanNodes, poseGraphEdges);
}

/* Save the metrics */
void SaveMetrics(const std::string& fileName)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();
    /* Convert all metrics to the property tree */
    const pt::ptree metricsTree = pMetricManager->ToPropertyTree();

    /* Write metrics to the JSON file */
    const std::string metricsFileName = fileName + ".metric.json";
    pt::write_json(metricsFileName, metricsTree);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << ' '
                  << "<Carmen log file name> "
                  << "<JSON Settings file name> "
                  << "[Output name]" << std::endl;
        return EXIT_FAILURE;
    }

    fs::path logFilePath { argv[1] };
    fs::path settingsFilePath { argv[2] };

    /* Determine the output file name */
    const bool hasValidFileName = logFilePath.has_stem() &&
                                  logFilePath.stem() != "." &&
                                  logFilePath.stem() != "..";
    fs::path outputFilePath { (argc == 4 || !hasValidFileName) ? argv[3] :
                              logFilePath.stem() };

    /* Load Carmen log file */
    std::vector<Sensor::SensorDataPtr> logData;
    LoadCarmenLog(logFilePath, logData);

    if (logData.empty())
        return EXIT_FAILURE;

    /* Load settings from JSON configuration file */
    Mapping::LidarGraphSlamPtr pLidarGraphSlam;
    LauncherSettings launcherSettings;

    if (!LoadSettings(settingsFilePath, pLidarGraphSlam, launcherSettings))
        return EXIT_FAILURE;

    /* Start the SLAM backend */
    pLidarGraphSlam->StartBackend();

    /* Setup gnuplot helper */
    auto pGnuplotHelper = launcherSettings.mGuiEnabled ?
        std::make_unique<IO::GnuplotHelper>() : nullptr;

    for (auto& sensorData : logData) {
        auto scanData = std::dynamic_pointer_cast<
            Sensor::ScanData<double>>(std::move(sensorData));

        if (scanData == nullptr)
            continue;

        /* Process the latest scan data */
        const bool mapUpdated = pLidarGraphSlam->ProcessScan(
            scanData, scanData->OdomPose());

        if (!launcherSettings.mGuiEnabled || !mapUpdated)
            continue;
        if (pLidarGraphSlam->ProcessCount() %
            launcherSettings.mDrawFrameInterval != 0)
            continue;

        /* Draw the current pose graph if necessary */
        DrawPoseGraph(pLidarGraphSlam, pGnuplotHelper);
    }

    /* Stop the SLAM backend */
    pLidarGraphSlam->StopBackend();

    /* Draw the final pose graph if necessary */
    if (launcherSettings.mGuiEnabled)
        DrawPoseGraph(pLidarGraphSlam, pGnuplotHelper);

    IO::MapSaver* const pMapSaver = IO::MapSaver::Instance();

    /* Retrieve a latest map that contains latest scans */
    RobotPose2D<double> latestMapPose;
    Mapping::GridMap latestMap;
    Mapping::NodeId latestMapNodeIdMin { Mapping::NodeId::Invalid };
    Mapping::NodeId latestMapNodeIdMax { Mapping::NodeId::Invalid };
    pLidarGraphSlam->GetLatestMap(latestMapPose, latestMap,
                                  latestMapNodeIdMin, latestMapNodeIdMax);

    /* Retrieve all pose graph nodes and edges */
    Mapping::IdMap<Mapping::LocalMapId,
        Mapping::LocalMapNode> localMapNodes;
    Mapping::IdMap<Mapping::NodeId,
        Mapping::ScanNode> scanNodes;
    std::vector<Mapping::PoseGraphEdge> poseGraphEdges;
    pLidarGraphSlam->GetPoseGraph(localMapNodes, scanNodes, poseGraphEdges);
    const Mapping::NodeId scanNodeIdMin = scanNodes.IdMin();
    const Mapping::NodeId scanNodeIdMax = scanNodes.IdMax();

    /* Build a global map that contains all local grid maps */
    RobotPose2D<double> globalMapPose;
    Mapping::GridMap globalMap;
    pLidarGraphSlam->GetGlobalMap(globalMapPose, globalMap,
                                  scanNodeIdMin, scanNodeIdMax);

    /* Save the global map, the pose graph, and the latest map */
    pMapSaver->SaveMap(
        globalMapPose, globalMap, &scanNodes,
        &scanNodeIdMin, &scanNodeIdMax, true, outputFilePath);
    pMapSaver->SavePoseGraph(
        localMapNodes, scanNodes, poseGraphEdges, outputFilePath);
    pMapSaver->SaveLatestMapAndScan(
        latestMapPose, latestMap, &scanNodes,
        &latestMapNodeIdMin, &latestMapNodeIdMax,
        nullptr, nullptr, true, outputFilePath);

    /* Save the metrics */
    SaveMetrics(outputFilePath);

    if (launcherSettings.mWaitForKey)
        std::getchar();

    return EXIT_SUCCESS;
}
