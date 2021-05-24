
/* util.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_UTIL_HPP
#define MY_LIDAR_GRAPH_SLAM_UTIL_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"

/* Constant for the Pi */
template <typename T>
inline constexpr T Pi = static_cast<T>(3.14159265358979323846);

/* Constant for the Pi divided by 2 */
template <typename T>
inline constexpr T PiHalf = static_cast<T>(1.57079632679489661923);

/* Constant for the reciprocal of Pi */
template <typename T>
inline constexpr T PiReciprocal = static_cast<T>(0.31830988618379067154);

/* Assert macro with a custom message (nullptr is allowed) */
#define XAssert(predicate, message) \
    (!(predicate) && AssertHandler(#predicate, __FILE__, __LINE__, message))
/* Assert macro with no custom message */
#define Assert(predicate) \
    XAssert(predicate, nullptr)

/* Assert function that is enabled in Release mode */
inline bool AssertHandler(const char* predicateExpression,
                          const char* fileName,
                          const int lineNumber,
                          const char* message)
{
    const char* customMessage = message != nullptr ? message : "";
    std::cerr << "Assertion failed: " << predicateExpression << '\n'
              << "File: " << fileName << ", "
              << "Line: " << lineNumber << ", "
              << "Message: " << customMessage << '\n';
    std::abort();
    return true;
}

namespace MyLidarGraphSlam {

/* Union type for data conversion */
union UInt32Conversion
{
    std::int32_t  mInt32Value;
    std::uint32_t mUInt32Value;
    float         mFloat32Value;
};

/* Union type for data conversion */
union UInt64Conversion
{
    std::uint64_t mUInt64Value;

    struct
    {
        std::int32_t mValue0;
        std::int32_t mValue1;
    } mInt32;

    struct
    {
        std::uint32_t mValue0;
        std::uint32_t mValue1;
    } mUInt32;

    struct
    {
        float mValue0;
        float mValue1;
    } mFloat32;
};

/* Reinterpret a 32-bit signed integer value (int) as
 * a 32-bit unsigned integer value (std::uint32_t) */
inline std::uint32_t Int32ToUInt32(const int value)
{
    UInt32Conversion dataConv;
    dataConv.mInt32Value = value;
    return dataConv.mUInt32Value;
}

/* Reinterpret a single-precision floating-point number (float) as
 * a 32-bit unsigned integer value (std::uint32_t) */
inline std::uint32_t FloatToUInt32(const float value)
{
    UInt32Conversion dataConv;
    dataConv.mFloat32Value = value;
    return dataConv.mUInt32Value;
}

/* Reinterpret a 32-bit unsigned integer value (std::uint32_t) as
 * a single-precision floating-point number (float) */
inline float UInt32ToFloat(const std::uint32_t value)
{
    UInt32Conversion dataConv;
    dataConv.mUInt32Value = value;
    return dataConv.mFloat32Value;
}

/* Pack two 32-bit unsigned integer values (std::uint32_t) and
 * convert to a 64-bit unsigned integer value (std::uint64_t) */
inline std::uint64_t PackUInt32(const std::uint32_t value0,
                                const std::uint32_t value1)
{
    UInt64Conversion dataConv;
    dataConv.mUInt32.mValue0 = value0;
    dataConv.mUInt32.mValue1 = value1;
    return dataConv.mUInt64Value;
}

/* Pack two single-precision floating-point number (float) and
 * convert to a 64-bit unsigned integer value (std::uint64_t) */
inline std::uint64_t PackFloat(const float value0, const float value1)
{
    UInt64Conversion dataConv;
    dataConv.mFloat32.mValue0 = value0;
    dataConv.mFloat32.mValue1 = value1;
    return dataConv.mUInt64Value;
}

/* Unpack a 64-bit unsigned integer value (std::uint64_t) and
 * return two 32-bit signed integer values (int) */
inline void UnpackInt32(const std::uint64_t packedValue,
                        std::int32_t& value0, std::int32_t& value1)
{
    UInt64Conversion dataConv;
    dataConv.mUInt64Value = packedValue;
    value0 = dataConv.mInt32.mValue0;
    value1 = dataConv.mInt32.mValue1;
}

/* Unpack a 64-bit unsigned integer value (std::uint64_t) and
 * return two 32-bit unsigned integer values (std::uint32_t) */
inline void UnpackUInt32(const std::uint64_t packedValue,
                         std::uint32_t& value0, std::uint32_t& value1)
{
    UInt64Conversion dataConv;
    dataConv.mUInt64Value = packedValue;
    value0 = dataConv.mUInt32.mValue0;
    value1 = dataConv.mUInt32.mValue1;
}

/* Unpack a 64-bit unsigned integer value (std::uint64_t) and
 * return two single-precision floating-point numbers (float) */
inline void UnpackFloat(const std::uint64_t packedValue,
                        float& value0, float& value1)
{
    UInt64Conversion dataConv;
    dataConv.mUInt64Value = packedValue;
    value0 = dataConv.mFloat32.mValue0;
    value1 = dataConv.mFloat32.mValue1;
}

/* Join elements in a container with a delimiter */
template <class C, class V = typename C::value_type>
std::string Join(const C& elements, const char* delimiter)
{
    std::ostringstream strStream;
    auto beginIt = std::begin(elements);
    auto endIt = std::end(elements);

    /* Print the elements if the container is not empty */
    if (beginIt != endIt) {
        std::copy(beginIt, std::prev(endIt),
                  std::ostream_iterator<V>(strStream, delimiter));
        beginIt = std::prev(endIt);
    }

    /* Print the last remaining element if the container is not empty */
    if (beginIt != endIt)
        strStream << *beginIt;

    return strStream.str();
}

/* Split a given string with a delimiter string */
std::vector<std::string> Split(const std::string& str,
                               const std::string& delimiter);

/* Split a given string with a delimiter character */
std::vector<std::string> Split(const std::string& str,
                               const char delimiter);

/* Convert strongly typed enum to integers */
template <typename T>
inline constexpr auto ToUnderlying(T enumValue) noexcept
{
    return static_cast<std::underlying_type_t<T>>(enumValue);
}

/* Convert rgb to unsigned 32-bit value */
inline std::uint32_t RGBToUInt32(
    std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    return static_cast<std::uint32_t>(
        (static_cast<std::uint32_t>(0)) |
        (static_cast<std::uint32_t>(r) << 16) |
        (static_cast<std::uint32_t>(g) << 8) |
        (static_cast<std::uint32_t>(b)));
}

/* Extract rgb components from unsigned 32-bit value */
inline std::uint8_t UInt32ToR(std::uint32_t rgb)
{
    return static_cast<std::uint8_t>((rgb & 0x00FF0000) >> 16);
}

/* Extract rgb components from unsigned 32-bit value */
inline std::uint8_t UInt32ToG(std::uint32_t rgb)
{
    return static_cast<std::uint8_t>((rgb & 0x0000FF00) >> 8);
}

/* Extract rgb components from unsigned 32-bit value */
inline std::uint8_t UInt32ToB(std::uint32_t rgb)
{
    return static_cast<std::uint8_t>((rgb & 0x000000FF));
}

/* Convert nanoseconds to milliseconds */
inline double ToMilliSeconds(std::int_least64_t nanoSec)
{
    const auto nanoSeconds = std::chrono::nanoseconds(nanoSec);
    const auto milliSeconds = std::chrono::duration_cast<
        std::chrono::milliseconds>(nanoSeconds);
    return milliSeconds.count();
}

/* Convert nanoseconds to microseconds */
inline double ToMicroSeconds(std::int_least64_t nanoSec)
{
    const auto nanoSeconds = std::chrono::nanoseconds(nanoSec);
    const auto microSeconds = std::chrono::duration_cast<
        std::chrono::microseconds>(nanoSeconds);
    return microSeconds.count();
}

/* Check if the number is power of 2 */
inline bool IsPowerOf2(int x)
{
    return (x != 0) && !(x & (x - 1));
}

/* Normalize the angle in radians from -pi to pi */
template <typename T, std::enable_if_t<
          std::is_arithmetic_v<T>, std::nullptr_t> = nullptr>
T NormalizeAngle(T theta)
{
    T normalizedTheta = std::fmod(theta, 2.0 * Pi<T>);

    if (normalizedTheta > Pi<T>)
        normalizedTheta -= 2.0 * Pi<T>;
    else if (normalizedTheta < -Pi<T>)
        normalizedTheta += 2.0 * Pi<T>;

    return normalizedTheta;
}

/* Normalize the angle in radians from -pi to pi */
template <typename T>
RobotPose2D<T> NormalizeAngle(const RobotPose2D<T>& robotPose)
{
    return RobotPose2D<T> { robotPose.mX,
                            robotPose.mY,
                            NormalizeAngle(robotPose.mTheta) };
}

/* Convert polar coordinate to cartesian coordinate */
template <typename T>
Point2D<T> ToCartesianCoordinate(T radius, T angle)
{
    return Point2D<T> { radius * std::cos(angle),
                        radius * std::sin(angle) };
}

/* Convert cartesian coordinate to polar coordinate */
template <typename T>
std::pair<double, double> ToPolarCoordinate(const Point2D<T>& point)
{
    double radius = std::sqrt(point.mX * point.mX + point.mY * point.mY);
    double angle = std::atan2(point.mY, point.mX);
    return std::make_pair(radius, angle);
}

/* Rotate a covariance matrix */
inline Eigen::Matrix3d RotateCovariance(
    const double rotationAngle,
    const Eigen::Matrix3d& covMat)
{
    const double cosTheta = std::cos(rotationAngle);
    const double sinTheta = std::sin(rotationAngle);

    /* Create a rotation matrix */
    Eigen::Matrix3d rotationMat;
    rotationMat << cosTheta, -sinTheta, 0.0,
                   sinTheta,  cosTheta, 0.0,
                   0.0,       0.0,      1.0;
    
    /* Rotate a covariance matrix */
    return rotationMat * covMat * rotationMat.transpose();
}

/* Convert a covariance matrix from world frame to robot frame */
inline Eigen::Matrix3d ConvertCovarianceFromWorldToLocal(
    const RobotPose2D<double>& poseToLocalFrame,
    const Eigen::Matrix3d& worldCovMat)
{
    return RotateCovariance(-poseToLocalFrame.mTheta, worldCovMat);
}

/* Convert a covariance matrix from local frame to world frame */
inline Eigen::Matrix3d ConvertCovarianceFromLocalToWorld(
    const RobotPose2D<double>& poseToLocalFrame,
    const Eigen::Matrix3d& localCovMat)
{
    return RotateCovariance(poseToLocalFrame.mTheta, localCovMat);
}

/* Convert radian to degree */
template <typename T>
inline constexpr T ConvertRadianToDegree(T theta)
{
    return theta * PiReciprocal<T> * 180.0;
}

/* Convert degree to radian */
template <typename T>
inline constexpr T ConvertDegreeToRadian(T theta)
{
    return theta * Pi<T> / 180.0;
}

/* Execute sliding window maximum problem */
template <typename T>
void SlidingWindowMax(std::function<T(int)> inFunc,
                      std::function<void(int, T)> outFunc,
                      int numOfElements,
                      int winSize)
{
    /* Source code was taken from https://www.geeksforgeeks.org/ and
     * slightly modified (zero padding added) */

    /* Create a double ended queue, idxQueue that will store indexes of
     * array elements
     * idxQueue will store indexes of useful elements in every window and
     * it will maintain decreasing order of values from front to rear in
     * idxQueue, i.e., inFunc(idxQueue.front()) to inFunc(idxQueue.back()) are
     * sorted in decreasing order */
    std::deque<int> idxQueue;

    int idxIn = 0;
    int idxOut = 0;

    /* Process the first winSize (or first window) elements of array */
    for (idxIn = 0; idxIn < winSize; ++idxIn) {
        /* For every element, the previous smaller elements are useless so
         * remove them from idxQueue */
        while ((!idxQueue.empty()) && inFunc(idxIn) >= inFunc(idxQueue.back()))
            idxQueue.pop_back();
        
        /* Add new element at rear of idxQueue */
        idxQueue.push_back(idxIn);
    }

    /* Now idxQueue.front() contains the index of the maximum element
     * in the first window */

    /* Process the rest of the elements */
    for (; idxIn < numOfElements; ++idxIn) {
        /* The element at the front of the queue is the maximum element of
         * the previous window */
        outFunc(idxOut++, inFunc(idxQueue.front()));

        /* Remove the elements which are out of the current window */
        while ((!idxQueue.empty()) && idxQueue.front() <= idxIn - winSize)
            idxQueue.pop_front();
        
        /* Remove all elements smaller than the current element */
        while ((!idxQueue.empty()) && inFunc(idxIn) >= inFunc(idxQueue.back()))
            idxQueue.pop_back();
        
        /* Add the current element at the rear of the queue */
        idxQueue.push_back(idxIn);
    }

    /* Repeat the last elements */
    for (; idxOut < numOfElements; ++idxOut)
        outFunc(idxOut, inFunc(idxQueue.front()));
}

/* Execute bresenham algorithm */
template <typename T>
void Bresenham(const Point2D<T>& startIdx,
               const Point2D<T>& endIdx,
               std::vector<Point2D<T>>& indices)
{
    /* Clear the indices */
    indices.clear();

    int deltaX = endIdx.mX - startIdx.mX;
    int deltaY = endIdx.mY - startIdx.mY;
    int stepX = (deltaX < 0) ? -1 : 1;
    int stepY = (deltaY < 0) ? -1 : 1;
    int nextX = startIdx.mX;
    int nextY = startIdx.mY;

    deltaX = std::abs(deltaX * 2);
    deltaY = std::abs(deltaY * 2);

    /* Append the start cell index */
    indices.emplace_back(nextX, nextY);

    /* Execute Bresenham algorithm */
    if (deltaX > deltaY) {
        int err = deltaY - deltaX / 2;

        while (nextX != endIdx.mX) {
            if (err >= 0) {
                nextY += stepY;
                err -= deltaX;
            }
            nextX += stepX;
            err += deltaY;
            indices.emplace_back(nextX, nextY);
        }
    } else {
        int err = deltaX - deltaY / 2;

        while (nextY != endIdx.mY) {
            if (err >= 0) {
                nextX += stepX;
                err -= deltaY;
            }
            nextY += stepY;
            err += deltaX;
            indices.emplace_back(nextX, nextY);
        }
    }
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_UTIL_HPP */
