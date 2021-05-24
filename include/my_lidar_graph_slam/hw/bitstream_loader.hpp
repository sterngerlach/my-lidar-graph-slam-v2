
/* bitstream_loader.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_BITSTREAM_LOADER_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_BITSTREAM_LOADER_HPP

#include <cstdint>
#include <string>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * BitstreamFileInfo struct holds the necessary information for the
 * bitstream file
 */
struct BitstreamFileInfo
{
    /* Absolute path to the original bitstream file */
    std::string mFilePath;
    /* Bitstream firmware file name (e.g. design0.bit) */
    std::string mFirmwareFileName;
    /* Absolute path to the firmware (e.g. /lib/firmware/design0.bit) */
    std::string mFirmwarePath;
};

/*
 * BitstreamHeaderInfo struct holds the necessary information for the
 * bitstream file header
 */
struct BitstreamHeaderInfo
{
    /* Design name (e.g. design0_wrapper) */
    std::string mDesignName;
    /* User id string (e.g. UserID=0xFFFFFFFF) */
    std::string mUserId;
    /* Version name (e.g. Version=2019.2) */
    std::string mVersionName;
    /* Part name (e.g. 7z020clg400) */
    std::string mPartName;
    /* Date (e.g. 2020/11/20) */
    std::string mDate;
    /* Time (e.g. 13:27:09) */
    std::string mTime;
    /* Offset to the bitstream data (in bytes) */
    std::size_t mOffsetToBitstream;
    /* Size of the bitstream data (in bytes) */
    std::size_t mBitstreamSize;
};

/*
 * BitstreamLoader class loads the specified bitstream file to
 * enable the hardware accelerators
 */
class BitstreamLoader final
{
public:
    /* Constructor */
    BitstreamLoader() = default;
    /* Destructor */
    ~BitstreamLoader() = default;

    /* Copy constructor (disabled) */
    BitstreamLoader(const BitstreamLoader&) = delete;
    /* Copy assignment operator (disabled) */
    BitstreamLoader& operator=(const BitstreamLoader&) = delete;

    /* Move constructor */
    BitstreamLoader(BitstreamLoader&&) = default;
    /* Move assignment operator */
    BitstreamLoader& operator=(BitstreamLoader&&) = default;

    /* Load the specified bitstream file */
    bool Load(const std::string& filePath);

private:
    /* Parse the header of the bitstream file */
    bool ParseHeader(const char* pFileData,
                     const std::size_t fileSize,
                     BitstreamHeaderInfo& headerInfo);

    /* Swap the data bytes in the bitstream */
    void SwapDataBytes(char* pBitstreamData,
                       const std::size_t bitstreamSize);

    /* Get the absolute path to the firmware */
    void GetFirmwarePath(const std::string& sourceFilePath,
                         std::string& fileName,
                         std::string& firmwarePath);

    /* Copy the byte-swapped bitstream to the driver firmware directory */
    bool CopyBitstreamFile(const char* pBitstreamData,
                           const std::size_t bitstreamSize,
                           const std::string& firmwarePath);

    /* Write the bitstream type flags */
    bool WriteBitstreamFlags();
    /* Write the bitstream file name */
    bool WriteBitstreamFileName(const std::string& firmwareFileName);

private:
    /* Absolute path to the bitstream type flags */
    static constexpr const char* FpgaManagerFlags =
        "/sys/class/fpga_manager/fpga0/flags";
    /* Absolute path to the bitstream firmware */
    static constexpr const char* FpgaManagerFirmware =
        "/sys/class/fpga_manager/fpga0/firmware";
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_BITSTREAM_LOADER_HPP */
