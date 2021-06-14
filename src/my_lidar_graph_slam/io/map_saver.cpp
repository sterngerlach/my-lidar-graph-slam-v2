
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

#include "my_lidar_graph_slam/bresenham.hpp"
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
    const Mapping::GridMap& gridMap)
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

    /* Compute the size of the cropped grid map */
    const BoundingBox<int> croppedBox =
        mapSaveQuery.mGridMap->CroppedBoundingBox();

    /* Initialize the image */
    gil::rgb8_image_t mapImage { croppedBox.Width(), croppedBox.Height() };
    const gil::rgb8_view_t& mapImageView = gil::view(mapImage);
    gil::fill_pixels(mapImageView, gil::rgb8_pixel_t(192, 192, 192));

    /* Draw the grid cells to the image */
    this->DrawMap(mapImageView, mapSaveQuery.mGridMap, croppedBox);

    /* Draw the trajectory (pose graph nodes) of the robot */
    if (!mapSaveQuery.mTrajectoryPoses.empty())
        this->DrawTrajectory(mapImageView, mapSaveQuery, croppedBox);

    /* Draw the scans to the image */
    if (!mapSaveQuery.mScans.empty())
        this->DrawScan(mapImageView, mapSaveQuery, croppedBox);

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
        this->SaveMapMetadata(mapSaveQuery, croppedBox, metadataFileName);
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
    const Mapping::GridMap& gridMap,
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
    const Mapping::GridMap& latestMap,
    const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
    const NodeId* pNodeIdMin,
    const NodeId* pNodeIdMax,
    const RobotPose2D<double>* pScanGlobalPose,
    const Sensor::ScanDataPtr<double>& pScanData,
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
    if (pScanGlobalPose != nullptr)
        saveQuery.AppendScan(*pScanGlobalPose, pScanData,
                             gil::rgb8_pixel_t(0, 0, 255));

    /* Save the map image */
    return this->SaveMap(saveQuery);
}

/* Save precomputed grid maps stored in a local grid map */
bool MapSaver::SavePrecomputedGridMaps(
    const RobotPose2D<double>& mapPose,
    const std::vector<Mapping::ConstMap>& precompMaps,
    const bool saveMetadata,
    const std::string& fileName) const
{
    /* Convert the precomputed grid map and save */
    for (std::size_t i = 0; i < precompMaps.size(); ++i) {
        /* Create the occupancy grid map from the coarser grid map */
        const Mapping::GridMap gridMap { precompMaps[i] };

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
void MapSaver::DrawMap(const gil::rgb8_view_t& mapImageView,
                       const Mapping::GridMap* gridMap,
                       const BoundingBox<int>& boundingBox) const
{
    /* Check that the grid map has the same size as the image */
    Assert(mapImageView.width() == boundingBox.Width());
    Assert(mapImageView.height() == boundingBox.Height());

    const auto toGrayScale = [](const std::uint8_t value) {
        return gil::rgb8_pixel_t(value, value, value); };
    const auto unknownProb = gridMap->UnknownProbability();

    /* Draw the grid cells to the image */
    for (int row = boundingBox.mMin.mY; row < boundingBox.mMax.mY; ++row) {
        for (int col = boundingBox.mMin.mX; col < boundingBox.mMax.mX; ++col) {
            /* Get the probability value */
            const auto prob = gridMap->ProbabilityOr(row, col, unknownProb);

            /* Skip if the grid cell is unknown */
            if (prob == unknownProb)
                continue;

            /* Convert the probability to the pixel intensity */
            const auto imageRow = static_cast<std::ptrdiff_t>(
                row - boundingBox.mMin.mY);
            const auto imageCol = static_cast<std::ptrdiff_t>(
                col - boundingBox.mMin.mX);
            const auto pixelValue = static_cast<std::uint8_t>(
                (1.0 - prob) * 255.0);

            /* Set the pixel intensity */
            mapImageView(imageCol, imageRow) = toGrayScale(pixelValue);
        }
    }
}

/* Draw the trajectory lines to the image */
void MapSaver::DrawTrajectory(const gil::rgb8_view_t& mapImageView,
                              const MapSaveQuery& mapSaveQuery,
                              const BoundingBox<int>& boundingBox) const
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
    const auto scaledGeometry =
        gridMap->Geometry().ScaledGeometry(SubpixelScale);
    Point2D<int> prevIdx = scaledGeometry.PositionToIndex(
        firstPose.mX, firstPose.mY);

    for (std::size_t i = 1; i < trajectoryPoses.size(); ++i) {
        const RobotPose2D<double> trajectoryPose =
            InverseCompound(gridMapPose, trajectoryPoses[i]);
        const Point2D<int> scaledIdx = scaledGeometry.PositionToIndex(
            trajectoryPose.mX, trajectoryPose.mY);
        std::vector<Point2D<int>> lineIndices;
        BresenhamScaled(prevIdx, scaledIdx, SubpixelScale, lineIndices);

        for (const auto& interpolatedIdx : lineIndices) {
            if (interpolatedIdx.mX < boundingBox.mMin.mX ||
                interpolatedIdx.mX > boundingBox.mMax.mX - lineWidth ||
                interpolatedIdx.mY < boundingBox.mMin.mY ||
                interpolatedIdx.mY > boundingBox.mMax.mY - lineWidth)
                continue;

            const int x = interpolatedIdx.mX - boundingBox.mMin.mX;
            const int y = interpolatedIdx.mY - boundingBox.mMin.mY;
            const auto& subView = gil::subimage_view(
                mapImageView, x, y, lineWidth, lineWidth);
            gil::fill_pixels(subView, trajectoryColor);
        }

        prevIdx = scaledIdx;
    }
}

/* Draw the scans obtained at the specified node to the image */
void MapSaver::DrawScan(const boost::gil::rgb8_view_t& mapImageView,
                        const MapSaveQuery& mapSaveQuery,
                        const BoundingBox<int>& boundingBox) const
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
            gridMap->PositionToIndex(localScanPose.mX, localScanPose.mY);

        if (scanPoseIdx.mX >= boundingBox.mMin.mX &&
            scanPoseIdx.mX <= boundingBox.mMax.mX - scanPointSize &&
            scanPoseIdx.mY >= boundingBox.mMin.mY &&
            scanPoseIdx.mY <= boundingBox.mMax.mY - scanPointSize) {
            const int x = scanPoseIdx.mX - boundingBox.mMin.mX;
            const int y = scanPoseIdx.mY - boundingBox.mMin.mY;
            const auto& subView = gil::subimage_view(
                mapImageView, x, y, scanPointSize, scanPointSize);
            gil::fill_pixels(subView, scanColors[i]);
        }

        const RobotPose2D<double> globalSensorPose =
            Compound(scanPoses[i], scans[i]->RelativeSensorPose());
        const RobotPose2D<double> localSensorPose =
            InverseCompound(gridMapPose, globalSensorPose);
        const std::size_t numOfScans = scans[i]->NumOfScans();

        for (std::size_t j = 0; j < numOfScans; ++j) {
            /* Calculate the grid cell index */
            const Point2D<double> localHitPoint =
                scans[i]->HitPoint(localSensorPose, j);
            const Point2D<int> hitPointIdx =
                gridMap->PositionToIndex(localHitPoint.mX, localHitPoint.mY);

            if (hitPointIdx.mX < boundingBox.mMin.mX ||
                hitPointIdx.mX > boundingBox.mMax.mX - scanPointSize ||
                hitPointIdx.mY < boundingBox.mMin.mY ||
                hitPointIdx.mY > boundingBox.mMax.mY - scanPointSize)
                continue;

            /* Draw the scan point to the image */
            const int x = hitPointIdx.mX - boundingBox.mMin.mX;
            const int y = hitPointIdx.mY - boundingBox.mMin.mY;
            const auto& subView = gil::subimage_view(
                mapImageView, x, y, scanPointSize, scanPointSize);
            gil::fill_pixels(subView, scanColors[i]);
        }
    }
}

/* Save the map metadata as JSON format */
void MapSaver::SaveMapMetadata(const MapSaveQuery& mapSaveQuery,
                               const BoundingBox<int>& boundingBox,
                               const std::string& fileName) const
{
    pt::ptree jsonMetadata;

    /* Write the grid map metadata */
    jsonMetadata.put("GlobalPose.X", mapSaveQuery.mGridMapPose.mX);
    jsonMetadata.put("GlobalPose.Y", mapSaveQuery.mGridMapPose.mY);
    jsonMetadata.put("GlobalPose.Theta", mapSaveQuery.mGridMapPose.mTheta);

    const auto* pGridMap = mapSaveQuery.mGridMap;

    jsonMetadata.put("Resolution", pGridMap->Resolution());
    jsonMetadata.put("Log2BlockSize", pGridMap->Log2BlockSize());
    jsonMetadata.put("BlockSize", pGridMap->BlockSize());

    const int rows = boundingBox.Height();
    const int cols = boundingBox.Width();
    const int blockRows = (rows + pGridMap->BlockSize() - 1)
                          >> pGridMap->Log2BlockSize();
    const int blockCols = (cols + pGridMap->BlockSize() - 1)
                          >> pGridMap->Log2BlockSize();
    const double height = pGridMap->Resolution() * rows;
    const double width = pGridMap->Resolution() * cols;

    jsonMetadata.put("Rows", rows);
    jsonMetadata.put("Cols", cols);
    jsonMetadata.put("BlockRows", blockRows);
    jsonMetadata.put("BlockCols", blockCols);
    jsonMetadata.put("Height", height);
    jsonMetadata.put("Width", width);

    /* `posMin` represents the position of the (0, 0) grid cell
     * in the map-local coordinate frame, from which the pose of the
     * (0, 0) grid cell in the global coordinate frame can be computed
     * using the pose of the grid map `mapSaveQuery.mGridMapPose` */
    const auto posMin = pGridMap->IndexToPosition(
        boundingBox.mMin.mY, boundingBox.mMin.mX);
    /* `posMax` represents the position of the corner grid cell
     * in the map-local coordinate frame */
    const auto posMax = pGridMap->IndexToPosition(
        boundingBox.mMax.mY, boundingBox.mMax.mX);

    jsonMetadata.put("PositionMin.X", posMin.mX);
    jsonMetadata.put("PositionMin.Y", posMin.mY);
    jsonMetadata.put("PositionMax.X", posMax.mX);
    jsonMetadata.put("PositionMax.Y", posMax.mY);

    /* Save the map metadata as JSON format */
    pt::write_json(fileName, jsonMetadata);

    return;
}

} /* namespace IO */
} /* namespace MyLidarGraphSlam */
