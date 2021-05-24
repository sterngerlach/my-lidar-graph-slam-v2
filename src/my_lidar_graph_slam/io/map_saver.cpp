
/* map_saver.cpp */

#include "my_lidar_graph_slam/io/map_saver.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map.hpp"

/* Declare namespaces for convenience */
namespace gil = boost::gil;
namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {
namespace IO {

/*
 * MapSaveQuery struct implementation
 */

/* Set the grid map information */
MapSaveQuery& MapSaveQuery::GridMap(
    const RobotPose2D<double>& gridMapPose,
    const Mapping::GridMapType& gridMap)
{
    this->mGridMapPose = gridMapPose;
    this->mGridMap = &gridMap;
    return *this;
}

/* Append the trajectory pose */
MapSaveQuery& MapSaveQuery::AppendTrajectory(
    const RobotPose2D<double>& robotPose)
{
    this->mTrajectoryPoses.push_back(robotPose);
    return *this;
}

/* Set the color of the trajectory lines */
MapSaveQuery& MapSaveQuery::TrajectoryColor(
    const gil::rgb8_pixel_t trajectoryColor)
{
    this->mTrajectoryColor = trajectoryColor;
    return *this;
}

/* Set the width of the trajectory lines */
MapSaveQuery& MapSaveQuery::TrajectoryLineWidth(
    const int trajectoryLineWidth)
{
    this->mTrajectoryLineWidth = trajectoryLineWidth;
    return *this;
}

/* Append the scan data */
MapSaveQuery& MapSaveQuery::AppendScan(
    const RobotPose2D<double>& scanPose,
    const Sensor::ScanDataPtr<double>& scanData,
    const gil::rgb8_pixel_t scanColor)
{
    this->mScanPoses.push_back(scanPose);
    this->mScans.push_back(scanData.get());
    this->mScanColors.push_back(scanColor);
    return *this;
}

/* Set the size of the scan points */
MapSaveQuery& MapSaveQuery::ScanPointSize(const int scanPointSize)
{
    this->mScanPointSize = scanPointSize;
    return *this;
}

/* Set the flag to save the metadata */
MapSaveQuery& MapSaveQuery::SaveMetadata(const bool saveMetadata)
{
    this->mSaveMetadata = saveMetadata;
    return *this;
}

/* Set the file name */
MapSaveQuery& MapSaveQuery::FileName(const std::string& fileName)
{
    this->mFileName = fileName;
    return *this;
}

/* Get the MapSaver instance */
MapSaver* MapSaver::Instance()
{
    static MapSaver theInstance;
    return &theInstance;
}

/* Save the grid map as an image file */
bool MapSaver::SaveMap(const MapSaveQuery& mapSaveQuery) const
{
    /* Current implementation requires the grid map */
    Assert(mapSaveQuery.mGridMap != nullptr);

    /* Compute the map size to be written to the image */
    Point2D<int> patchIdxMin;
    Point2D<int> patchIdxMax;
    Point2D<int> gridCellIdxMin;
    Point2D<int> gridCellIdxMax;
    Point2D<int> mapSizeInPatches;
    Point2D<int> mapSizeInGridCells;
    Point2D<double> mapSizeInMeters;
    mapSaveQuery.mGridMap->ComputeActualMapSize(
        patchIdxMin, patchIdxMax, gridCellIdxMin, gridCellIdxMax,
        mapSizeInPatches, mapSizeInGridCells, mapSizeInMeters);

    /* Initialize the image */
    gil::rgb8_image_t mapImage { mapSizeInGridCells.mX,
                                 mapSizeInGridCells.mY };
    const gil::rgb8_view_t& mapImageView = gil::view(mapImage);
    gil::fill_pixels(mapImageView, gil::rgb8_pixel_t(192, 192, 192));

    /* Draw the grid cells to the image */
    this->DrawMap(mapImageView, mapSaveQuery,
                  patchIdxMin, patchIdxMax, mapSizeInPatches);

    /* Draw the trajectory (pose graph nodes) of the robot */
    if (!mapSaveQuery.mTrajectoryPoses.empty())
        this->DrawTrajectory(
            mapImageView, mapSaveQuery,
            gridCellIdxMin, gridCellIdxMax, mapSizeInGridCells);

    /* Draw the scans to the image */
    if (!mapSaveQuery.mScans.empty())
        this->DrawScan(
            mapImageView, mapSaveQuery,
            gridCellIdxMin, gridCellIdxMax, mapSizeInGridCells);

    /* Save the map as PNG image
     * Image should be flipped upside down */
    try {
        const std::string pngFileName = mapSaveQuery.mFileName + ".png";
#if BOOST_VERSION <= 106700
        gil::png_write_view(pngFileName,
                            gil::flipped_up_down_view(mapImageView));
#else
        gil::write_view(pngFileName,
                        gil::flipped_up_down_view(mapImageView),
                        gil::png_tag());
#endif
    } catch (const std::ios_base::failure& e) {
        std::cerr << "std::ios_base::failure occurred: " << e.what() << ' '
                  << "(Error code: " << e.code() << ")" << std::endl;
        return false;
    }

    if (!mapSaveQuery.mSaveMetadata)
        return true;

    /* Save the map metadata as JSON format */
    try {
        const std::string metadataFileName = mapSaveQuery.mFileName + ".json";
        this->SaveMapMetadata(
            mapSaveQuery.mGridMapPose, mapSaveQuery.mGridMap->Resolution(),
            mapSaveQuery.mGridMap->PatchSize(),
            mapSizeInPatches, mapSizeInGridCells, mapSizeInMeters,
            mapSaveQuery.mGridMap->LocalMinPos(),
            mapSaveQuery.mGridMap->LocalMaxPos(),
            metadataFileName);
    } catch (const pt::json_parser_error& e) {
        std::cerr << "boost::property_tree::json_parser_error occurred: "
                  << e.what() << ' '
                  << "(" << e.filename() << ", Line " << e.line() << ")"
                  << std::endl;
        return false;
    }

    return true;
}

/* Save the entire map */
bool MapSaver::SaveMap(
    const RobotPose2D<double>& gridMapPose,
    const Mapping::GridMapType& gridMap,
    const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
    const NodeId* pNodeIdMin,
    const NodeId* pNodeIdMax,
    const bool saveMetadata,
    const std::string& fileName) const
{
    /* Create the query information */
    MapSaveQuery saveQuery;
    saveQuery.GridMap(gridMapPose, gridMap)
             .SaveMetadata(saveMetadata)
             .FileName(fileName)
             .TrajectoryColor(gil::rgb8_pixel_t(255, 0, 0))
             .TrajectoryLineWidth(2);

    /* Draw the trajectory lines if necessary */
    if (pTrajectoryNodes != nullptr) {
        const auto firstNodeIt = pTrajectoryNodes->find(*pNodeIdMin);
        const auto lastNodeIt = pTrajectoryNodes->find(*pNodeIdMax);
        const auto endNodeIt = std::next(lastNodeIt);
        const auto nodeRange = pTrajectoryNodes->RangeFromIterator(
            firstNodeIt, endNodeIt);

        for (const auto& [scanNodeId, scanNode] : nodeRange)
            saveQuery.AppendTrajectory(scanNode.mGlobalPose);
    }

    /* Global map accommodates all local grid maps acquired */
    /* Save the map image and metadata */
    return this->SaveMap(saveQuery);
}

/* Save the pose graph as JSON format */
bool MapSaver::SavePoseGraph(
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>& scanNodes,
    const std::vector<PoseGraphEdge>& poseGraphEdges,
    const std::string& fileName) const
{
    pt::ptree jsonPoseGraph;

    /* Write the local map nodes */
    pt::ptree localMapNodesTree;

    for (const auto& [localMapId, localMapNode] : localMapNodes) {
        pt::ptree nodeInfo;

        /* Convert to single-precision floating point */
        const RobotPose2D<float> globalPose =
            static_cast<RobotPose2D<float>>(localMapNode.mGlobalPose);

        /* Write the data for the local map node */
        nodeInfo.put("Id", localMapNode.mLocalMapId.mId);
        nodeInfo.put("GlobalPose.X", globalPose.mX);
        nodeInfo.put("GlobalPose.Y", globalPose.mY);
        nodeInfo.put("GlobalPose.Theta", globalPose.mTheta);

        /* Append the local map node */
        localMapNodesTree.push_back(std::make_pair("", nodeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.LocalMapNodes", localMapNodesTree);

    /* Write the scan nodes */
    pt::ptree scanNodesTree;

    for (const auto& [nodeId, scanNode] : scanNodes) {
        pt::ptree nodeInfo;

        /* Convert to single-precision floating point */
        const RobotPose2D<float> localPose =
            static_cast<RobotPose2D<float>>(scanNode.mLocalPose);
        const RobotPose2D<float> globalPose =
            static_cast<RobotPose2D<float>>(scanNode.mGlobalPose);

        /* Write the data for the scan node */
        nodeInfo.put("Id", scanNode.mNodeId.mId);
        nodeInfo.put("LocalMapId", scanNode.mLocalMapId.mId);
        nodeInfo.put("LocalPose.X", localPose.mX);
        nodeInfo.put("LocalPose.Y", localPose.mY);
        nodeInfo.put("LocalPose.Theta", localPose.mTheta);
        nodeInfo.put("TimeStamp", scanNode.mScanData->TimeStamp());
        nodeInfo.put("GlobalPose.X", globalPose.mX);
        nodeInfo.put("GlobalPose.Y", globalPose.mY);
        nodeInfo.put("GlobalPose.Theta", globalPose.mTheta);

        /* Append the scan node */
        scanNodesTree.push_back(std::make_pair("", nodeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.ScanNodes", scanNodesTree);

    /* Write the pose graph edges */
    pt::ptree poseGraphEdgesTree;

    for (const auto& edge : poseGraphEdges) {
        pt::ptree edgeInfo;

        /* Convert to single-precision floating point */
        const RobotPose2D<float> relativePose =
            static_cast<RobotPose2D<float>>(edge.mRelativePose);

        /* Write the data for the edge */
        edgeInfo.put("LocalMapNodeId", edge.mLocalMapNodeId.mId);
        edgeInfo.put("ScanNodeId", edge.mScanNodeId.mId);
        edgeInfo.put("EdgeType", static_cast<int>(edge.mEdgeType));
        edgeInfo.put("ConstraintType", static_cast<int>(edge.mConstraintType));
        edgeInfo.put("RelativePose.X", relativePose.mX);
        edgeInfo.put("RelativePose.Y", relativePose.mY);
        edgeInfo.put("RelativePose.Theta", relativePose.mTheta);

        /* Store elements of information matrix and covariance matrix
         * (upper triangular elements only) */
        pt::ptree infoMatElements;
        pt::ptree covMatElements;
        const Eigen::Matrix3d& infoMat = edge.mInformationMat;
        const Eigen::Matrix3d covMat = infoMat.inverse();

        for (Eigen::Index i = 0; i < infoMat.rows(); ++i) {
            for (Eigen::Index j = i; j < infoMat.cols(); ++j) {
                pt::ptree infoMatElement;
                pt::ptree covMatElement;
                infoMatElement.put_value(static_cast<float>(infoMat(i, j)));
                covMatElement.put_value(static_cast<float>(covMat(i, j)));
                infoMatElements.push_back(std::make_pair("", infoMatElement));
                covMatElements.push_back(std::make_pair("", covMatElement));
            }
        }

        edgeInfo.add_child("InformationMatrix", infoMatElements);
        edgeInfo.add_child("CovarianceMatrix", covMatElements);

        /* Append the pose graph edge */
        poseGraphEdgesTree.push_back(std::make_pair("", edgeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.Edges", poseGraphEdgesTree);

    /* Save the pose graph as JSON format */
    try {
        const std::string poseGraphFileName = fileName + ".posegraph.json";
        pt::write_json(poseGraphFileName, jsonPoseGraph);
    } catch (const pt::json_parser_error& e) {
        std::cerr << "boost::property_tree::json_parser_error occurred: "
                  << e.what()
                  << "(" << e.filename()
                  << ", Line " << e.line() << ")" << std::endl;
        return false;
    }

    return true;
}

/* Save local maps individually */
bool MapSaver::SaveLocalMaps(
    const IdMap<LocalMapId, Mapping::LocalMap>& localMaps,
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
    const bool saveMetadata,
    const std::string& fileName) const
{
    Assert(!localMaps.empty());
    Assert(!localMapNodes.empty());
    Assert(localMaps.size() == localMapNodes.size());

    /* Save local maps individually as PNG images */
    for (const auto& [localMapId, localMap] : localMaps) {
        /* Retrieve the local map node */
        const auto& localMapNode = localMapNodes.at(localMapId);
        /* Determine the file name */
        const std::string localMapFileName =
            fileName + "-local-map-" + std::to_string(localMapId.mId);

        /* Create the query information */
        MapSaveQuery saveQuery;
        saveQuery.GridMap(localMapNode.mGlobalPose, localMap.mMap)
                 .SaveMetadata(saveMetadata)
                 .FileName(localMapFileName)
                 .TrajectoryColor(gil::rgb8_pixel_t(255, 0, 0))
                 .TrajectoryLineWidth(2);

        /* Draw the trajectory lines if necessary */
        if (pTrajectoryNodes != nullptr) {
            const auto firstNodeIt =
                pTrajectoryNodes->find(localMap.mScanNodeIdMin);
            const auto lastNodeIt =
                pTrajectoryNodes->find(localMap.mScanNodeIdMax);
            const auto endNodeIt = std::next(lastNodeIt);
            const auto nodeRange =
                pTrajectoryNodes->RangeFromIterator(firstNodeIt, endNodeIt);

            for (const auto& [scanNodeId, scanNode] : nodeRange)
                saveQuery.AppendTrajectory(scanNode.mGlobalPose);
        }

        /* Save the map image and metadata */
        if (!this->SaveMap(saveQuery))
            return false;
    }

    return true;
}

/* Save the map and the scan */
bool MapSaver::SaveLocalMapAndScan(
    const RobotPose2D<double>& localMapPose,
    const Mapping::LocalMap& localMap,
    const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
    const ScanNode* pScanNode,
    const bool saveMetadata,
    const std::string& fileName) const
{
    const std::string localMapFileName =
        fileName + "-local-map-" + std::to_string(localMap.mId.mId);

    /* Create the query information */
    MapSaveQuery saveQuery;
    saveQuery.GridMap(localMapPose, localMap.mMap)
             .SaveMetadata(saveMetadata)
             .FileName(localMapFileName)
             .TrajectoryColor(gil::rgb8_pixel_t(255, 0, 0))
             .TrajectoryLineWidth(2)
             .ScanPointSize(2);

    /* Draw the trajectory lines if necessary */
    if (pTrajectoryNodes != nullptr) {
        const auto firstNodeIt =
            pTrajectoryNodes->find(localMap.mScanNodeIdMin);
        const auto lastNodeIt =
            pTrajectoryNodes->find(localMap.mScanNodeIdMax);
        const auto endNodeIt = std::next(lastNodeIt);
        const auto nodeRange =
            pTrajectoryNodes->RangeFromIterator(firstNodeIt, endNodeIt);

        for (const auto& [scanNodeId, scanNode] : nodeRange)
            saveQuery.AppendTrajectory(scanNode.mGlobalPose);
    }

    /* Draw the scan points if necessary */
    if (pScanNode != nullptr)
        saveQuery.AppendScan(pScanNode->mGlobalPose, pScanNode->mScanData,
                             gil::rgb8_pixel_t(0, 0, 255));

    /* Save the map image */
    return this->SaveMap(saveQuery);
}

/* Save the latest map and the scan */
bool MapSaver::SaveLatestMapAndScan(
    const RobotPose2D<double>& latestMapPose,
    const Mapping::GridMapType& latestMap,
    const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
    const NodeId* pNodeIdMin,
    const NodeId* pNodeIdMax,
    const ScanNode* pScanNode,
    const bool saveMetadata,
    const std::string& fileName) const
{
    const std::string latestMapFileName = fileName + "-latest-map";

    /* Create the query information */
    MapSaveQuery saveQuery;
    saveQuery.GridMap(latestMapPose, latestMap)
             .SaveMetadata(saveMetadata)
             .FileName(latestMapFileName)
             .TrajectoryColor(gil::rgb8_pixel_t(255, 0, 0))
             .TrajectoryLineWidth(2)
             .ScanPointSize(2);

    /* Draw the trajectory lines if necessary */
    if (pTrajectoryNodes != nullptr) {
        const auto firstNodeIt = pTrajectoryNodes->find(*pNodeIdMin);
        const auto lastNodeIt = pTrajectoryNodes->find(*pNodeIdMax);
        const auto endNodeIt = std::next(lastNodeIt);
        const auto nodeRange = pTrajectoryNodes->RangeFromIterator(
            firstNodeIt, endNodeIt);

        for (const auto& [scanNodeId, scanNode] : nodeRange)
            saveQuery.AppendTrajectory(scanNode.mGlobalPose);
    }

    /* Draw the scan points if necessary */
    if (pScanNode != nullptr)
        saveQuery.AppendScan(pScanNode->mGlobalPose, pScanNode->mScanData,
                             gil::rgb8_pixel_t(0, 0, 255));

    /* Save the map image */
    return this->SaveMap(saveQuery);
}

/* Save precomputed grid maps stored in a local grid map */
bool MapSaver::SavePrecomputedGridMaps(
    const RobotPose2D<double>& mapPose,
    const std::vector<Mapping::ConstMapType>& precompMaps,
    const bool saveMetadata,
    const std::string& fileName) const
{
    /* Convert the precomputed grid map and save */
    for (std::size_t i = 0; i < precompMaps.size(); ++i) {
        /* Retrieve the coarser grid map */
        const auto& precompMap = precompMaps[i];

        /* Create the occupancy grid map */
        Mapping::GridMapType gridMap =
            Mapping::GridMapType::CreateSameSizeMap(precompMap);

        const double unknownVal = precompMap.UnknownValue();
        const int numOfGridCellsX = precompMap.NumOfGridCellsX();
        const int numOfGridCellsY = precompMap.NumOfGridCellsY();

        /* Copy the occupancy probability values */
        for (int y = 0; y < numOfGridCellsY; ++y) {
            for (int x = 0; x < numOfGridCellsX; ++x) {
                const double mapValue = precompMap.Value(x, y, unknownVal);

                if (mapValue != unknownVal)
                    gridMap.Update(x, y, mapValue);
            }
        }

        const std::string precompMapFileName =
            fileName + "-precomp-map-" + std::to_string(i);

        /* Create the query information */
        MapSaveQuery saveQuery;
        saveQuery.GridMap(mapPose, gridMap)
                 .SaveMetadata(saveMetadata)
                 .FileName(precompMapFileName);

        if (!this->SaveMap(saveQuery))
            return false;
    }

    return true;
}

/* Draw the grid cells to the image */
void MapSaver::DrawMap(
    const gil::rgb8_view_t& mapImageView,
    const MapSaveQuery& mapSaveQuery,
    const Point2D<int>& patchIdxMin,
    const Point2D<int>& patchIdxMax,
    const Point2D<int>& mapSizeInPatches) const
{
    /* Retrieve the grid map */
    const auto* pGridMap = mapSaveQuery.mGridMap;

    /* Draw the patches to the image */
    for (int y = 0; y < mapSizeInPatches.mY; ++y) {
        for (int x = 0; x < mapSizeInPatches.mX; ++x) {
            const auto& patch = pGridMap->PatchAt(
                patchIdxMin.mX + x, patchIdxMin.mY + y);

            if (!patch.IsAllocated())
                continue;

            /* Draw the grid cells in the patch */
            for (int yy = 0; yy < pGridMap->PatchSize(); ++yy) {
                for (int xx = 0; xx < pGridMap->PatchSize(); ++xx) {
                    const double gridCellValue = patch.At(xx, yy).Value();

                    /* If the occupancy probability value is less than or
                     * equal to zero, then the grid cell is not yet observed
                     * and is in unknown state (GridCell::Unknown is zero) */
                    if (gridCellValue <= 0.0 || gridCellValue > 1.0)
                        continue;

                    const std::ptrdiff_t idxX = static_cast<std::ptrdiff_t>(
                        x * pGridMap->PatchSize() + xx);
                    const std::ptrdiff_t idxY = static_cast<std::ptrdiff_t>(
                        y * pGridMap->PatchSize() + yy);
                    const std::uint8_t grayScale = static_cast<std::uint8_t>(
                        (1.0 - gridCellValue) * 255.0);

                    mapImageView(idxX, idxY) =
                        gil::rgb8_pixel_t(grayScale, grayScale, grayScale);
                }
            }
        }
    }
}

/* Draw the trajectory lines to the image */
void MapSaver::DrawTrajectory(
    const gil::rgb8_view_t& mapImageView,
    const MapSaveQuery& mapSaveQuery,
    const Point2D<int>& gridCellIdxMin,
    const Point2D<int>& gridCellIdxMax,
    const Point2D<int>& mapSizeInGridCells) const
{
    /* Retrieve the necessary information */
    const auto& trajectoryPoses = mapSaveQuery.mTrajectoryPoses;
    const auto trajectoryColor = mapSaveQuery.mTrajectoryColor;
    const auto lineWidth = mapSaveQuery.mTrajectoryLineWidth;
    const auto& gridMapPose = mapSaveQuery.mGridMapPose;
    const auto& gridMap = mapSaveQuery.mGridMap;

    if (trajectoryPoses.size() < 2)
        return;

    /* Draw the trajectory lines to the image */
    const RobotPose2D<double> firstPose =
        InverseCompound(gridMapPose, trajectoryPoses.front());
    Point2D<int> prevGridCellIdx =
        gridMap->LocalPosToGridCellIndex(firstPose.mX, firstPose.mY);

    for (std::size_t i = 1; i < trajectoryPoses.size(); ++i) {
        const RobotPose2D<double> trajectoryPose =
            InverseCompound(gridMapPose, trajectoryPoses[i]);
        const Point2D<int> gridCellIdx =
            gridMap->LocalPosToGridCellIndex(
                trajectoryPose.mX, trajectoryPose.mY);
        std::vector<Point2D<int>> lineIndices;
        Bresenham(prevGridCellIdx, gridCellIdx, lineIndices);

        for (const auto& interpolatedIdx : lineIndices) {
            if (interpolatedIdx.mX < gridCellIdxMin.mX ||
                interpolatedIdx.mX > gridCellIdxMax.mX - lineWidth ||
                interpolatedIdx.mY < gridCellIdxMin.mY ||
                interpolatedIdx.mY > gridCellIdxMax.mY - lineWidth)
                continue;

            const int x = interpolatedIdx.mX - gridCellIdxMin.mX;
            const int y = interpolatedIdx.mY - gridCellIdxMin.mY;
            const auto& subView = gil::subimage_view(
                mapImageView, x, y, lineWidth, lineWidth);
            gil::fill_pixels(subView, trajectoryColor);
        }

        prevGridCellIdx = gridCellIdx;
    }
}

/* Draw the scans obtained at the specified node to the image */
void MapSaver::DrawScan(
    const boost::gil::rgb8_view_t& mapImageView,
    const MapSaveQuery& mapSaveQuery,
    const Point2D<int>& gridCellIdxMin,
    const Point2D<int>& gridCellIdxMax,
    const Point2D<int>& mapSizeInGridCells) const
{
    const auto& scanPoses = mapSaveQuery.mScanPoses;
    const auto& scans = mapSaveQuery.mScans;
    const auto& scanColors = mapSaveQuery.mScanColors;
    const auto scanPointSize = mapSaveQuery.mScanPointSize;
    const auto& gridMapPose = mapSaveQuery.mGridMapPose;
    const auto& gridMap = mapSaveQuery.mGridMap;

    for (std::size_t i = 0; i < scans.size(); ++i) {
        /* Draw the scan pose (in a map-local coordinate frame) */
        const RobotPose2D<double> localScanPose =
            InverseCompound(gridMapPose, scanPoses[i]);
        const Point2D<int> scanPoseIdx =
            gridMap->LocalPosToGridCellIndex(
                localScanPose.mX, localScanPose.mY);

        if (scanPoseIdx.mX >= gridCellIdxMin.mX &&
            scanPoseIdx.mX <= gridCellIdxMax.mX - scanPointSize &&
            scanPoseIdx.mY >= gridCellIdxMin.mY &&
            scanPoseIdx.mY <= gridCellIdxMax.mX - scanPointSize) {
            const int x = scanPoseIdx.mX - gridCellIdxMin.mX;
            const int y = scanPoseIdx.mY - gridCellIdxMin.mY;
            const auto& subView = gil::subimage_view(
                mapImageView, x, y, scanPointSize, scanPointSize);
            gil::fill_pixels(subView, scanColors[i]);
        }

        const RobotPose2D<double> globalSensorPose =
            Compound(scanPoses[i], scans[i]->RelativeSensorPose());
        const RobotPose2D<double> localSensorPose =
            InverseCompound(gridMapPose, globalSensorPose);
        const std::size_t numOfScans = scans[i]->NumOfScans();

        for (std::size_t i = 0; i < numOfScans; ++i) {
            /* Calculate the grid cell index */
            const Point2D<double> localHitPoint =
                scans[i]->HitPoint(localSensorPose, i);
            const Point2D<int> hitPointIdx =
                gridMap->LocalPosToGridCellIndex(
                    localHitPoint.mX, localHitPoint.mY);

            if (hitPointIdx.mX < gridCellIdxMin.mX ||
                hitPointIdx.mX > gridCellIdxMax.mX - scanPointSize ||
                hitPointIdx.mY < gridCellIdxMin.mY ||
                hitPointIdx.mY > gridCellIdxMax.mY - scanPointSize)
                continue;

            /* Draw the scan point to the image */
            const int x = hitPointIdx.mX - gridCellIdxMin.mX;
            const int y = hitPointIdx.mY - gridCellIdxMin.mY;
            const auto& subView = gil::subimage_view(
                mapImageView, x, y, scanPointSize, scanPointSize);
            gil::fill_pixels(subView, scanColors[i]);
        }
    }
}

/* Save the map metadata as JSON format */
void MapSaver::SaveMapMetadata(
    const RobotPose2D<double>& gridMapPose,
    const double mapResolution,
    const int patchSize,
    const Point2D<int>& mapSizeInPatches,
    const Point2D<int>& mapSizeInGridCells,
    const Point2D<double>& mapSizeInMeters,
    const Point2D<double>& localMinPos,
    const Point2D<double>& localMaxPos,
    const std::string& fileName) const
{
    pt::ptree jsonMetadata;

    /* Write the map metadata */
    jsonMetadata.put("Map.GlobalPose.X", gridMapPose.mX);
    jsonMetadata.put("Map.GlobalPose.Y", gridMapPose.mY);
    jsonMetadata.put("Map.GlobalPose.Theta", gridMapPose.mTheta);
    jsonMetadata.put("Map.Resolution", mapResolution);
    jsonMetadata.put("Map.PatchSize", patchSize);

    jsonMetadata.put("Map.WidthInPatches", mapSizeInPatches.mX);
    jsonMetadata.put("Map.HeightInPatches", mapSizeInPatches.mY);
    jsonMetadata.put("Map.WidthInGridCells", mapSizeInGridCells.mX);
    jsonMetadata.put("Map.HeightInGridCells", mapSizeInGridCells.mY);
    jsonMetadata.put("Map.WidthInMeters", mapSizeInMeters.mX);
    jsonMetadata.put("Map.HeightInMeters", mapSizeInMeters.mY);

    jsonMetadata.put("Map.LocalMinPos.X", localMinPos.mX);
    jsonMetadata.put("Map.LocalMinPos.Y", localMinPos.mY);
    jsonMetadata.put("Map.LocalMaxPos.X", localMaxPos.mX);
    jsonMetadata.put("Map.LocalMaxPos.Y", localMaxPos.mY);

    /* Save the map metadata as JSON format */
    pt::write_json(fileName, jsonMetadata);

    return;
}

} /* namespace IO */
} /* namespace MyLidarGraphSlam */
