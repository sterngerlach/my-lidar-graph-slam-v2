
/* slam_client.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_NETWORK_SLAM_CLIENT_HPP
#define MY_LIDAR_GRAPH_SLAM_NETWORK_SLAM_CLIENT_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "my_lidar_graph_slam/network/data_types.hpp"

namespace MyLidarGraphSlam {
namespace Network {

enum class MessageType : std::uint32_t
{
    StopSignal,
    PoseArray,
    Scan,
    GridMapParams,
};

class SlamClient
{
public:
    /* Constructor */
    SlamClient(const std::string& serverAddress,
               const std::uint16_t serverPort);
    /* Destructor */
    ~SlamClient() = default;

    /* Copy constructor (disabled) */
    SlamClient(const SlamClient&) = delete;
    /* Copy assignment operator (disabled) */
    SlamClient& operator=(const SlamClient&) = delete;
    /* Move constructor (disabled) */
    SlamClient(SlamClient&&) = delete;
    /* Move assignment operator (disabled) */
    SlamClient& operator=(SlamClient&&) = delete;

    /* Connect to a server */
    bool ConnectToServer();
    /* Disconnect from a server */
    bool DisconnectFromServer();

    /* Send a scan message */
    bool SendScan(const Scan2D& scan);
    /* Send a pose array message */
    bool SendPoseArray(const std::vector<TimedPose2D>& poseArray);
    /* Send a grid map parameter message */
    bool SendGridMapParams(const GridMapParams& params);

private:
    /* Handshake between a client and a server */
    bool Handshake();
    /* Send a stop signal */
    bool SendStopSignal();
    /* Send a message header (message type) */
    bool SendMessageType(MessageType msgType);

private:
    /* Address of the server */
    std::string   mServerAddress;
    /* Port of the server */
    std::uint16_t mServerPort;
    /* Socket of the client */
    int           mClientSock;
    /* Flag activated when connected to the server */
    bool          mIsConnected;
};

} /* namespace Network */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_NETWORK_SLAM_CLIENT_HPP */
