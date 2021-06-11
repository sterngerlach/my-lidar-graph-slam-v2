
/* map_saver.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP
#define MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP

#include <memory>
#include <string>
#include <vector>

/* Definitions to prevent compile errors
 * int_p_NULL is removed in libpng 1.4 */
#define png_infopp_NULL     (png_infopp)NULL
#define int_p_NULL          (int*)NULL
#define png_bytep_NULL      (png_bytep)NULL

#include <boost/version.hpp>

#if BOOST_VERSION <= 106700
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_io.hpp>
#elif BOOST_VERSION <= 106800
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png.hpp>
#else
#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>
#endif

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace IO {

/*
 * MapSaveQuery struct stores the necessary information for saving
 * a given grid map as an image
 */
struct MapSaveQuery
{
    /* Type declaration for convenience */
    using GridMapPtr = const Mapping::GridMap*;
    using PoseVector = std::vector<RobotPose2D<double>>;
    using ScanVector = std::vector<const Sensor::ScanData<double>*>;
    using Color = boost::gil::rgb8_pixel_t;
    using ColorVector = std::vector<boost::gil::rgb8_pixel_t>;

    /* Constructor */
    MapSaveQuery() : mGridMapPose(RobotPose2D<double>::Zero),
                     mGridMap(nullptr),
                     mTrajectoryColor(255, 0, 0),
                     mTrajectoryLineWidth(2),
                     mScanPointSize(2),
                     mSaveMetadata(false),
                     mFileName() { }

    /* Destructor */
    ~MapSaveQuery() = default;

    /* Copy constructor (disabled) */
    MapSaveQuery(const MapSaveQuery&) = delete;
    /* Copy assignment operator (disabled) */
    MapSaveQuery& operator=(const MapSaveQuery&) = delete;

    /* Move constructor */
    MapSaveQuery(MapSaveQuery&&) = default;
    /* Move assignment operator */
    MapSaveQuery& operator=(MapSaveQuery&&) = default;

    /* Set the grid map information */
    MapSaveQuery& GridMap(const RobotPose2D<double>& gridMapPose,
                          const Mapping::GridMap& gridMap);

    /* Append the trajectory pose */
    MapSaveQuery& AppendTrajectory(const RobotPose2D<double>& robotPose);
    /* Set the color of the trajectory lines */
    MapSaveQuery& TrajectoryColor(const Color trajectoryColor);
    /* Set the width of the trajectory lines */
    MapSaveQuery& TrajectoryLineWidth(const int trajectoryLineWidth);

    /* Append the scan data */
    MapSaveQuery& AppendScan(const RobotPose2D<double>& scanPose,
                             const Sensor::ScanDataPtr<double>& scanData,
                             const Color scanColor);
    /* Set the size of the scan points */
    MapSaveQuery& ScanPointSize(const int scanPointSize);

    /* Set the flag to save the metadata */
    MapSaveQuery& SaveMetadata(const bool saveMetadata);
    /* Set the file name */
    MapSaveQuery& FileName(const std::string& fileName);

    /* Pose of the grid map in a world coordinate frame */
    RobotPose2D<double> mGridMapPose;
    /* Grid map */
    GridMapPtr          mGridMap;

    /* Collection of the trajectory poses (in a world coordinate frame) */
    PoseVector          mTrajectoryPoses;
    /* Color of the trajectory lines */
    Color               mTrajectoryColor;
    /* Width of the trajectory lines */
    int                 mTrajectoryLineWidth;

    /* Collection of the scan poses (in a world coordinate frame) */
    PoseVector          mScanPoses;
    /* Collection of the scan data */
    ScanVector          mScans;
    /* Collection of the scan colors */
    ColorVector         mScanColors;
    /* Size of the scan points */
    int                 mScanPointSize;

    /* Flag to save the metadata */
    bool                mSaveMetadata;
    /* File name */
    std::string         mFileName;
};

class MapSaver
{
public:
    /* Type definitions */
    using GridMapBuilderPtr = std::shared_ptr<Mapping::GridMapBuilder>;
    using PoseGraphPtr = std::shared_ptr<Mapping::PoseGraph>;
    using ScanPtr = Sensor::ScanDataPtr<double>;

    template <typename IdType, typename DataType>
    using IdMap = Mapping::IdMap<IdType, DataType>;

    using NodeId = Mapping::NodeId;
    using ScanNode = Mapping::ScanNode;
    using LocalMapId = Mapping::LocalMapId;
    using LocalMapNode = Mapping::LocalMapNode;
    using PoseGraphEdge = Mapping::PoseGraphEdge;

private:
    /* Constructor */
    MapSaver() = default;
    /* Destructor */
    ~MapSaver() = default;

public:
    /* Copy constructor (disabled) */
    MapSaver(const MapSaver&) = delete;
    /* Copy assignment operator (disabled) */
    MapSaver& operator=(const MapSaver&) = delete;
    /* Move constructor (disabled) */
    MapSaver(MapSaver&&) = delete;
    /* Move assignment operator (disabled) */
    MapSaver& operator=(MapSaver&&) = delete;

    /* Get the MapSaver singleton instance */
    static MapSaver* Instance();

    /* Save the grid map as an image file */
    bool SaveMap(const MapSaveQuery& mapSaveQuery) const;

    /* Save the entire map */
    bool SaveMap(
        const RobotPose2D<double>& gridMapPose,
        const Mapping::GridMap& gridMap,
        const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
        const NodeId* pNodeIdMin,
        const NodeId* pNodeIdMax,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the pose graph as JSON format */
    bool SavePoseGraph(
        const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        const IdMap<NodeId, ScanNode>& scanNodes,
        const std::vector<PoseGraphEdge>& poseGraphEdges,
        const std::string& fileName) const;

    /* Save local maps individually */
    bool SaveLocalMaps(
        const IdMap<LocalMapId, Mapping::LocalMap>& localMaps,
        const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the map and the scan */
    bool SaveLocalMapAndScan(
        const RobotPose2D<double>& localMapPose,
        const Mapping::LocalMap& localMap,
        const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
        const ScanNode* pScanNode,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the latest map and the scan */
    bool SaveLatestMapAndScan(
        const RobotPose2D<double>& latestMapPose,
        const Mapping::GridMap& latestMap,
        const IdMap<NodeId, ScanNode>* pTrajectoryNodes,
        const NodeId* pNodeIdMin,
        const NodeId* pNodeIdMax,
        const RobotPose2D<double>* pScanGlobalPose,
        const Sensor::ScanDataPtr<double>& pScanData,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save precomputed grid maps stored in a local grid map */
    bool SavePrecomputedGridMaps(
        const RobotPose2D<double>& mapPose,
        const std::vector<Mapping::ConstMap>& precompMaps,
        const bool saveMetadata,
        const std::string& fileName) const;

private:
    /* Draw the grid cells to the image */
    void DrawMap(const boost::gil::rgb8_view_t& mapImageView,
                 const Mapping::GridMap* gridMap,
                 const BoundingBox<int>& boundingBox) const;

    /* Draw the trajectory lines to the image */
    void DrawTrajectory(const boost::gil::rgb8_view_t& mapImageView,
                        const MapSaveQuery& mapSaveQuery,
                        const BoundingBox<int>& boundingBox) const;

    /* Draw the scans obtained at the specified node to the image */
    void DrawScan(const boost::gil::rgb8_view_t& mapImageView,
                  const MapSaveQuery& mapSaveQuery,
                  const BoundingBox<int>& boundingBox) const;

    /* Save the map metadata as JSON format */
    void SaveMapMetadata(const MapSaveQuery& mapSaveQuery,
                         const BoundingBox<int>& boundingBox,
                         const std::string& fileName) const;
};

} /* namespace IO */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP */
