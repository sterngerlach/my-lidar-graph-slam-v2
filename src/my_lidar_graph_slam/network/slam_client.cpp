
/* slam_client.cpp */

#include "my_lidar_graph_slam/network/slam_client.hpp"
#include "my_lidar_graph_slam/util.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "This code works only on a little endian machine"
#endif

namespace MyLidarGraphSlam {
namespace Network {

union Union64
{
    std::uint8_t  mU8[8];
    std::uint64_t mU64;
    double        mDouble;
};

union Union32
{
    std::uint8_t mU8[4];
    std::int32_t mI32;
    float        mFloat;
};

/* 64-bit version of htonl() */
std::uint64_t hton64(const std::uint64_t value)
{
    Union64 x;
    x.mU64 = value;
    std::swap(x.mU8[0], x.mU8[7]);
    std::swap(x.mU8[1], x.mU8[6]);
    std::swap(x.mU8[2], x.mU8[5]);
    std::swap(x.mU8[3], x.mU8[4]);
    return x.mU64;
}

/* 64-bit version of ntohl() */
std::uint64_t ntoh64(const std::uint64_t value)
{
    return hton64(value);
}

/* 64-bit version of htonl() for double */
double htond(const double value)
{
    Union64 x;
    x.mDouble = value;
    std::swap(x.mU8[0], x.mU8[7]);
    std::swap(x.mU8[1], x.mU8[6]);
    std::swap(x.mU8[2], x.mU8[5]);
    std::swap(x.mU8[3], x.mU8[4]);
    return x.mDouble;
}

/* 64-bit version of ntohl() for double */
double ntohd(const double value)
{
    return htond(value);
}

/* htonl() for 32-bit signed integer */
std::int32_t htonI32(const std::int32_t value)
{
    Union32 x;
    x.mI32 = value;
    std::swap(x.mU8[0], x.mU8[3]);
    std::swap(x.mU8[1], x.mU8[2]);
    return x.mI32;
}

/* ntohl() for 32-bit signed integer */
std::int32_t ntohI32(const std::int32_t value)
{
    return htonI32(value);
}

/* Send all data in the specified buffer */
bool SendAll(int socket, const void* buffer, std::size_t length)
{
    auto* srcBuffer = reinterpret_cast<const std::uint8_t*>(buffer);
    ssize_t bytesSent = 0;
    ssize_t remainingLength = static_cast<ssize_t>(length);

    while (remainingLength > 0) {
        if ((bytesSent = send(socket, srcBuffer, remainingLength, 0)) == -1)
            return false;

        srcBuffer += bytesSent;
        remainingLength -= bytesSent;
    }

    return true;
}

/* Receive all data to the specified buffer */
bool RecvAll(int socket, void* buffer, std::size_t length)
{
    auto* dstBuffer = reinterpret_cast<std::uint8_t*>(buffer);
    ssize_t bytesRecv = 0;
    ssize_t remainingLength = static_cast<ssize_t>(length);

    while (remainingLength > 0) {
        if ((bytesRecv = recv(socket, dstBuffer, remainingLength, 0)) == -1)
            return false;

        dstBuffer += bytesRecv;
        remainingLength -= bytesRecv;
    }

    return true;
}

/* Send a 32-bit unsigned integer */
bool SendU32(int socket, std::uint32_t value)
{
    /* Convert to the network byte order */
    const std::uint32_t sentValue = htonl(value);
    return SendAll(socket, &sentValue, sizeof(sentValue));
}

/* Receive a 32-bit unsigned integer */
bool RecvU32(int socket, std::uint32_t& value)
{
    std::uint32_t recvValue;
    bool recvResult;

    /* Convert to the host byte order */
    if ((recvResult = RecvAll(socket, &recvValue, sizeof(recvValue))))
        value = ntohl(recvValue);

    return recvResult;
}

/* Send a 32-bit signed integer */
bool SendI32(int socket, std::int32_t value)
{
    /* Convert to the network byte order */
    const std::int32_t sentValue = htonI32(value);
    return SendAll(socket, &sentValue, sizeof(sentValue));
}

/* Send a double-precision floating-point value */
bool SendDouble(int socket, double value)
{
    /* Convert to the network byte order */
    const double sentValue = htond(value);
    return SendAll(socket, &sentValue, sizeof(sentValue));
}

/* Send an array of double-precision floating-point values */
bool SendDoubleArray(int socket, const std::vector<double>& array)
{
    /* Convert to the network byte order */
    std::vector<double> sentArray;
    sentArray.resize(array.size());
    std::transform(array.begin(), array.end(), sentArray.begin(),
                   [](const double value) { return htond(value); });

    const std::size_t numOfBytes = sentArray.size() * sizeof(double);
    return SendAll(socket, sentArray.data(), numOfBytes);
}

/* Constructor */
SlamClient::SlamClient(const std::string& serverAddress,
                       const std::uint16_t serverPort) :
    mServerAddress(serverAddress),
    mServerPort(serverPort),
    mClientSock(-1),
    mIsConnected(false)
{
}

/* Connect to a server */
bool SlamClient::ConnectToServer()
{
    Assert(!this->mIsConnected);

    /* Create a socket */
    this->mClientSock = socket(AF_INET, SOCK_STREAM, 0);
    if (this->mClientSock == -1) {
        std::cerr << "socket() failed: " << strerror(errno) << '\n';
        return false;
    }

    /* Connect to a server */
    struct sockaddr_in sockAddr;
    std::memset(&sockAddr, 0, sizeof(sockAddr));
    sockAddr.sin_family = AF_INET;
    sockAddr.sin_port = htons(this->mServerPort);
    sockAddr.sin_addr.s_addr = inet_addr(this->mServerAddress.c_str());

    if (connect(this->mClientSock, reinterpret_cast<sockaddr*>(&sockAddr),
                static_cast<socklen_t>(sizeof(sockAddr))) == -1) {
        std::cerr << "connect() failed: " << strerror(errno) << '\n';
        return false;
    }

    this->mIsConnected = true;
    std::cerr << "Connected to the server: " << this->mServerAddress
              << " (port: " << this->mServerPort << ") ...\n";

    /* Handshake between a client and a server */
    if (!this->Handshake())
        return false;

    return true;
}

/* Disconnect from a server */
bool SlamClient::DisconnectFromServer()
{
    Assert(this->mIsConnected);

    /* Send a stop signal */
    if (!this->SendStopSignal())
        return false;

    /* Shut down a connection */
    if (shutdown(this->mClientSock, SHUT_RDWR) == -1) {
        std::cerr << "shutdown() failed: " << strerror(errno) << '\n';
        return false;
    }

    /* Close a connection */
    if (close(this->mClientSock) == -1) {
        std::cerr << "close() failed: " << strerror(errno) << '\n';
        return false;
    }

    this->mClientSock = -1;
    this->mIsConnected = false;
    std::cerr << "Disconnected from the server: " << this->mServerAddress
              << " (port: " << this->mServerPort << ") ...\n";

    return true;
}

/* Send a scan message */
bool SlamClient::SendScan(const Scan2D& scan)
{
    Assert(this->mIsConnected);

    if (!this->SendMessageType(MessageType::Scan))
        return false;

    if (scan.mRanges.size() != scan.mAngles.size()) {
        std::cerr << "Number of ranges and angles must be the same\n";
        return false;
    }

    /* Send the number of scans */
    const auto numOfScans = static_cast<std::uint32_t>(scan.mRanges.size());
    if (!SendU32(this->mClientSock, numOfScans))
        return false;

    /* Send a timestamp */
    if (!SendDouble(this->mClientSock, scan.mTime))
        return false;
    /* Send a sensor pose relative to a robot */
    if (!SendDouble(this->mClientSock, scan.mSensorPose.mX))
        return false;
    if (!SendDouble(this->mClientSock, scan.mSensorPose.mY))
        return false;
    if (!SendDouble(this->mClientSock, scan.mSensorPose.mTheta))
        return false;
    /* Send minimum and maximum ranges (angles) of the scan */
    if (!SendDouble(this->mClientSock, scan.mMinRange))
        return false;
    if (!SendDouble(this->mClientSock, scan.mMaxRange))
        return false;
    if (!SendDouble(this->mClientSock, scan.mMinAngle))
        return false;
    if (!SendDouble(this->mClientSock, scan.mMaxAngle))
        return false;
    /* Send ranges and angles */
    if (!SendDoubleArray(this->mClientSock, scan.mRanges))
        return false;
    if (!SendDoubleArray(this->mClientSock, scan.mAngles))
        return false;

    return true;
}

/* Send a pose array message */
bool SlamClient::SendPoseArray(const std::vector<TimedPose2D>& poseArray)
{
    Assert(this->mIsConnected);

    if (!this->SendMessageType(MessageType::PoseArray))
        return false;

    /* Send the number of poses */
    const auto numOfPoses = static_cast<std::uint32_t>(poseArray.size());
    if (!SendU32(this->mClientSock, numOfPoses))
        return false;

    /* Copy the poses to the buffer */
    std::vector<double> poseBuffer;
    poseBuffer.resize(numOfPoses * 4);

    for (std::size_t i = 0, j = 0; i < numOfPoses; ++i, j += 4) {
        poseBuffer[j + 0] = poseArray[i].mTime;
        poseBuffer[j + 1] = poseArray[i].mPose.mX;
        poseBuffer[j + 2] = poseArray[i].mPose.mY;
        poseBuffer[j + 3] = poseArray[i].mPose.mTheta;
    }

    /* Send the poses */
    if (!SendDoubleArray(this->mClientSock, poseBuffer))
        return false;

    return true;
}

/* Send a grid map parameter message */
bool SlamClient::SendGridMapParams(const GridMapParams& params)
{
    Assert(this->mIsConnected);

    if (!this->SendMessageType(MessageType::GridMapParams))
        return false;

    /* Send a grid map resolution */
    if (!SendDouble(this->mClientSock, params.mResolution))
        return false;
    /* Send a grid block size */
    if (!SendI32(this->mClientSock, params.mBlockSize))
        return false;
    /* Send a subpixel scale */
    if (!SendI32(this->mClientSock, params.mSubpixelScale))
        return false;
    /* Send minimum and maximum scan ranges */
    if (!SendDouble(this->mClientSock, params.mMinRange))
        return false;
    if (!SendDouble(this->mClientSock, params.mMaxRange))
        return false;
    /* Send probabilities for hit and missed grid cells */
    if (!SendDouble(this->mClientSock, params.mProbabilityHit))
        return false;
    if (!SendDouble(this->mClientSock, params.mProbabilityMiss))
        return false;
    /* Send odds for hit and missed grid cells */
    if (!SendDouble(this->mClientSock, params.mOddsHit))
        return false;
    if (!SendDouble(this->mClientSock, params.mOddsMiss))
        return false;

    return true;
}

/* Handshake between a client and a server */
bool SlamClient::Handshake()
{
    Assert(this->mIsConnected);

    /* Receive a message for synchronization */
    std::uint32_t syncMsgRecv;
    if (!RecvU32(this->mClientSock, syncMsgRecv)) {
        std::cerr << "Failed to receive a message for synchronization\n";
        return false;
    }
    
    /* Send a message for synchronization */
    std::uint32_t syncMsg = 1;
    if (!SendU32(this->mClientSock, syncMsg)) {
        std::cerr << "Failed to send a message for synchronization\n";
        return false;
    }

    return true;
}

/* Send a stop signal */
bool SlamClient::SendStopSignal()
{
    Assert(this->mIsConnected);
    return this->SendMessageType(MessageType::StopSignal);
}

/* Send a message header (message type) */
bool SlamClient::SendMessageType(MessageType msgType)
{
    Assert(this->mIsConnected);

    const auto value = static_cast<std::uint32_t>(msgType);
    if (!SendU32(this->mClientSock, value)) {
        std::cerr << "Failed to send a message type\n";
        return false;
    }

    return true;
}

} /* namespace Network */
} /* namespace MyLidarGraphSlam */
