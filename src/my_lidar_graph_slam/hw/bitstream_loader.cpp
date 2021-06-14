
/* bitstream_loader.cpp */

#include "my_lidar_graph_slam/hw/bitstream_loader.hpp"

#ifdef __GNUC__
#if (__GNUC__ >= 6) && (__GNUC__ < 8)
#include <experimental/filesystem>
#elif (__GNUC__ >= 8)
#include <filesystem>
#endif
#endif

#include <fstream>
#include <iostream>
#include <memory>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "my_lidar_graph_slam/util.hpp"

#ifdef __GNUC__
#if (__GNUC__ >= 6) && (__GNUC__ < 8)
namespace fs = std::experimental::filesystem;
#elif (__GNUC__ >= 8)
namespace fs = std::filesystem;
#endif
#endif

namespace MyLidarGraphSlam {
namespace Hardware {

/* Toggle between big-endian representation and little-endian
 * representation for a 16-bit unsigned integer */
std::uint16_t SwapBytes(const std::uint16_t value)
{
    const std::uint16_t value0 = (value & 0x00FF) << 8;
    const std::uint16_t value1 = (value & 0xFF00) >> 8;

    return value0 | value1;
}

/* Toggle between big-endian representation and little-endian
 * representation for a 32-bit unsigned integer */
std::uint32_t SwapBytes(const std::uint32_t value)
{
    const std::uint32_t value0 = (value & 0x000000FF) << 24;
    const std::uint32_t value1 = (value & 0x0000FF00) << 8;
    const std::uint32_t value2 = (value & 0x00FF0000) >> 8;
    const std::uint32_t value3 = (value & 0xFF000000) >> 24;

    return value0 | value1 | value2 | value3;
}

/* Read a 8-bit unsigned integer at a specified location */
std::uint8_t ReadUInt8(const char* bitstreamData,
                       const std::size_t dataOffset)
{
    const auto* ptr = reinterpret_cast<const std::uint8_t*>(
        bitstreamData + dataOffset);
    return *ptr;
}

/* Read a 16-bit unsigned integer at a specified location */
std::uint16_t ReadUInt16(const char* bitstreamData,
                         const std::size_t dataOffset)
{
    const auto* ptr = reinterpret_cast<const std::uint8_t*>(
        bitstreamData + dataOffset);

    std::uint16_t value = 0;
    value |= static_cast<std::uint16_t>(*ptr++);
    value |= static_cast<std::uint16_t>(*ptr++) << 8;

    return value;
}

/* Read a 32-bit unsigned integer at a specified location */
std::uint32_t ReadUInt32(const char* bitstreamData,
                         const std::size_t dataOffset)
{
    const auto* ptr = reinterpret_cast<const std::uint8_t*>(
        bitstreamData + dataOffset);

    std::uint32_t value = 0;
    value |= static_cast<std::uint32_t>(*ptr++);
    value |= static_cast<std::uint32_t>(*ptr++) << 8;
    value |= static_cast<std::uint32_t>(*ptr++) << 16;
    value |= static_cast<std::uint32_t>(*ptr++) << 24;

    return value;
}

/* Write a 16-bit unsigned integer at a specified location */
void WriteUInt16(char* bitstreamData,
                 const std::size_t dataOffset,
                 const std::uint16_t value)
{
    auto* ptr = reinterpret_cast<std::uint8_t*>(bitstreamData + dataOffset);

    *ptr++ = static_cast<std::uint8_t>((value & 0x00FF));
    *ptr++ = static_cast<std::uint8_t>((value & 0xFF00) >> 8);
}

/* Write a 32-bit unsigned integer at a specified location */
void WriteUInt32(char* bitstreamData,
                 const std::size_t dataOffset,
                 const std::uint32_t value)
{
    auto* ptr = reinterpret_cast<std::uint8_t*>(bitstreamData + dataOffset);

    *ptr++ = static_cast<std::uint8_t>((value & 0x000000FF));
    *ptr++ = static_cast<std::uint8_t>((value & 0x0000FF00) >> 8);
    *ptr++ = static_cast<std::uint8_t>((value & 0x00FF0000) >> 16);
    *ptr++ = static_cast<std::uint8_t>((value & 0xFF000000) >> 24);
}

/*
 * BitstreamLoader class implementation
 */

/* Load the specified bitstream file */
bool BitstreamLoader::Load(const std::string& filePath)
{
    /* Open the bitstream file */
    std::ifstream fileStream;
    fileStream.open(filePath, std::ios::in | std::ios::binary);

    if (!fileStream) {
        std::cerr << "Failed to open bitstream file: "
                  << filePath << '\n';
        return false;
    }

    /* Get the size of the specified bitstream file */
    fileStream.seekg(0, std::ios::end);
    const std::size_t fileSize = fileStream.tellg();
    fileStream.seekg(0, std::ios::beg);

    /* Copy the bitstream to the newly allocated buffer */
    std::unique_ptr<char[]> pFileData { new char[fileSize] };
    fileStream.read(pFileData.get(), fileSize);

    /* Close the bitstream file */
    fileStream.close();

    BitstreamFileInfo fileInfo;
    BitstreamHeaderInfo headerInfo;

    /* Get the absolute path to the bitstream file */
    fileInfo.mFilePath = fs::absolute(filePath);
    /* Get the file name and the absolute path to the firmware */
    this->GetFirmwarePath(filePath,
                          fileInfo.mFirmwareFileName,
                          fileInfo.mFirmwarePath);

    /* Parse the bitstream file header */
    if (!this->ParseHeader(pFileData.get(), fileSize, headerInfo)) {
        std::cerr << "Failed to parse the bitstream file header\n";
        return false;
    }

    /* Copy the bitstream part to the newly allocated buffer
     * to prevent the unaligned memory accesses */
    /* FIXME: Use std::memmove() here */
    std::unique_ptr<char[]> pBitstreamData {
        new char[headerInfo.mBitstreamSize] };
    std::memcpy(pBitstreamData.get(),
                pFileData.get() + headerInfo.mOffsetToBitstream,
                headerInfo.mBitstreamSize);
    /* Destroy the bitstream file data since it is not used anymore */
    pFileData.reset(nullptr);

    /* Swap the data bytes in the bitstream */
    this->SwapDataBytes(pBitstreamData.get(), headerInfo.mBitstreamSize);

    /* Copy the byte-swapped bitstream to the driver firmware directory */
    if (!this->CopyBitstreamFile(pBitstreamData.get(),
                                 headerInfo.mBitstreamSize,
                                 fileInfo.mFirmwarePath)) {
        std::cerr << "Failed to copy the byte-swapped bitstream data to "
                  << "the driver firmware directory\n";
        return false;
    }

    /* Write the bitstream type flags */
    if (!this->WriteBitstreamFlags()) {
        std::cerr << "Failed to write the bitstream type flags "
                  << "(0: full bitstream and not encrypted)\n";
        return false;
    }

    /* Write the bitstream firmware file name */
    if (!this->WriteBitstreamFileName(fileInfo.mFirmwareFileName)) {
        std::cerr << "Failed to write the bitstream file name\n";
        return false;
    }

    std::cerr << "Bitstream is successfully loaded\n"
              << "Original file path: " << fileInfo.mFilePath << '\n'
              << "Firmware file name: " << fileInfo.mFirmwareFileName << '\n'
              << "Firmware file path: " << fileInfo.mFirmwarePath << '\n';

    std::cerr << "Design name: " << headerInfo.mDesignName << '\n'
              << "Version name: " << headerInfo.mVersionName << '\n'
              << "Part name: " << headerInfo.mPartName << '\n'
              << "Date: " << headerInfo.mDate << '\n'
              << "Time: " << headerInfo.mTime << '\n';

    return true;
}

/* Parse the header of the bitstream file */
bool BitstreamLoader::ParseHeader(const char* pFileData,
                                  const std::size_t fileSize,
                                  BitstreamHeaderInfo& headerInfo)
{
    std::size_t dataOffset = 0;
    std::size_t fieldLength = 0;
    std::string fieldValue;
    std::uint8_t separatorField;
    bool parseFinished = false;

    /* Read a 2-byte length field (0x0009) and a magic number */
    fieldLength = SwapBytes(ReadUInt16(pFileData, dataOffset));
    dataOffset += (2 + fieldLength);

    /* Read a 2-byte length field (0x0001) and a magic number */
    fieldLength = SwapBytes(ReadUInt16(pFileData, dataOffset));
    dataOffset += 2;

    while (!parseFinished) {
        /* Read a separator token (0x61, 0x62, 0x63, 0x64, 0x65) */
        separatorField = ReadUInt8(pFileData, dataOffset);
        dataOffset += 1;

        /* Read a 2-byte length field if necessary */
        if (separatorField != 0x65) {
            fieldLength = SwapBytes(ReadUInt16(pFileData, dataOffset));
            dataOffset += 2;
            fieldValue.assign(pFileData + dataOffset, fieldLength);
            dataOffset += fieldLength;
        }

        if (separatorField == 0x61) {
            /* Read a design name field and split into multiple fields */
            const auto fieldElements = Split(fieldValue, ';');

            /* Check the number of the fields */
            if (fieldElements.size() != 3) {
                std::cerr << "Invalid design name field: "
                          << fieldValue << "\n";
                return false;
            }

            /* Get a design name (e.g. design0_wrapper) */
            headerInfo.mDesignName = fieldElements[0];
            /* Get an user id string (e.g. UserID=0xFFFFFFFF) */
            headerInfo.mUserId = fieldElements[1];
            /* Get a version name (e.g. Version=2019.2) */
            headerInfo.mVersionName = fieldElements[2];
        } else if (separatorField == 0x62) {
            /* Read a part name (e.g. 7z020clg400) */
            headerInfo.mPartName = fieldValue;
        } else if (separatorField == 0x63) {
            /* Read a date (e.g. 2020/11/20) */
            headerInfo.mDate = fieldValue;
        } else if (separatorField == 0x64) {
            /* Read a time (e.g. 13:27:09) */
            headerInfo.mTime = fieldValue;
        } else if (separatorField == 0x65) {
            /* Read a bitstream length (e.g. 4045564) */
            parseFinished = true;
            /* Read a 4-byte length field (e.g. 4045564) */
            fieldLength = SwapBytes(ReadUInt32(pFileData, dataOffset));
            dataOffset += 4;

            /* Bitstream length should be divisible by 4 */
            if (fieldLength % sizeof(std::uint32_t) != 0) {
                std::cerr << "Invalid bitstream length: "
                          << fieldLength << ", "
                          << "length should be divisible by 4\n";
                return false;
            }

            /* Bitstream length should be consistent with the file size */
            if (fieldLength + dataOffset != fileSize) {
                std::cerr << "Invalid bitstream length: "
                          << fieldLength << ", "
                          << "length is inconsistent with the offset to "
                          << "the bitstream data (" << dataOffset << ") "
                          << "and the bitstream file size ("
                          << fileSize << ")\n";
                return false;
            }

            /* Set the offset to the bitstream data (in bytes) */
            headerInfo.mOffsetToBitstream = dataOffset;
            /* Set the size of the bitstream data (in bytes) */
            headerInfo.mBitstreamSize = fieldLength;
        } else {
            /* Unknown field */
            std::cerr << "Unknown field: " << separatorField << '\n';
            return false;
        }
    }

    return true;
}

/* Swap the data bytes in the bitstream */
void BitstreamLoader::SwapDataBytes(char* pBitstreamData,
                                    const std::size_t bitstreamSize)
{
    for (std::size_t dataOffset = 0;
         dataOffset < bitstreamSize; dataOffset += 4) {
        /* Retrieve the 32-bit unsigned integer from the bitstream */
        const std::uint32_t value = ReadUInt32(pBitstreamData, dataOffset);
        /* Swap the bytes in the 32-bit unsigned integer */
        const std::uint32_t swappedValue = SwapBytes(value);
        /* Overwrite the bitstream with the byte-swapped value */
        WriteUInt32(pBitstreamData, dataOffset, swappedValue);
    }

    return;
}

/* Get the absolute path to the firmware */
void BitstreamLoader::GetFirmwarePath(const std::string& sourceFilePath,
                                      std::string& fileName,
                                      std::string& firmwarePath)
{
    /* Build the bitstream file path */
    const fs::path sourcePath = sourceFilePath;
    const fs::path baseName = sourcePath.stem().replace_extension(".bin");
    const fs::path basePath = "/lib/firmware";
    const fs::path destPath = basePath / baseName;

    fileName = baseName;
    firmwarePath = destPath;
}

/* Copy the byte-swapped bitstream to the driver firmware directory */
bool BitstreamLoader::CopyBitstreamFile(const char* pBitstreamData,
                                        const std::size_t bitstreamSize,
                                        const std::string& firmwarePath)
{
    /* Open the output bitstream file */
    std::ofstream fileStream;
    fileStream.open(firmwarePath,
                    std::ios::out | std::ios::binary | std::ios::trunc);

    if (!fileStream) {
        std::cerr << "Failed to open bitstream file: "
                  << firmwarePath << '\n';
        return false;
    }

    /* Write the byte-swapped bitstream to the file */
    fileStream.write(pBitstreamData, bitstreamSize);
    /* Close the output bitstream file */
    fileStream.close();

    return true;
}

/* Write the bitstream type flags */
bool BitstreamLoader::WriteBitstreamFlags()
{
    int flagsFd = -1;

    /* Open the FPGA Manager flags device file */
    if ((flagsFd = open(FpgaManagerFlags, O_WRONLY)) == -1) {
        std::cerr << "Failed to open the FPGA Manager flags: "
                  << FpgaManagerFlags << '\n';
        std::cerr << "Errno: " << std::strerror(errno) << '\n';
        return false;
    }

    /* Full bitstream is loaded and is not encrypted */
    const char* typeFlags = "0";
    const ssize_t flagsLength = static_cast<ssize_t>(std::strlen(typeFlags));

    /* Write the bitstream type flags */
    if (write(flagsFd, typeFlags, flagsLength) != flagsLength) {
        std::cerr << "Failed to write the bitstream type flags to "
                  << FpgaManagerFlags << '\n';
        std::cerr << "Errno: " << std::strerror(errno) << '\n';
        return false;
    }

    /* Close the FPGA Manager flags device file */
    if (close(flagsFd) == -1) {
        std::cerr << "Failed to close the FPGA Manager flags: "
                  << FpgaManagerFlags << '\n';
        std::cerr << "Errno: " << std::strerror(errno) << '\n';
        return false;
    }

    return true;
}

/* Write the bitstream file name */
bool BitstreamLoader::WriteBitstreamFileName(
    const std::string& firmwareFileName)
{
    int firmwareFd = -1;

    /* Open the FPGA Manager firmware device file */
    if ((firmwareFd = open(FpgaManagerFirmware, O_WRONLY)) == -1) {
        std::cerr << "Failed to open the FPGA Manager firmware: "
                  << FpgaManagerFirmware << '\n';
        std::cerr << "Errno: " << std::strerror(errno) << '\n';
        return false;
    }

    /* Write the bitstream file name */
    const char* pFileName = firmwareFileName.c_str();
    const ssize_t nameLength = static_cast<ssize_t>(firmwareFileName.length());

    if (write(firmwareFd, pFileName, nameLength) != nameLength) {
        std::cerr << "Failed to write the bitstream file name to "
                  << FpgaManagerFirmware << '\n';
        std::cerr << "Errno: " << std::strerror(errno) << '\n';
        return false;
    }

    /* Close the FPGA Manager firmware device file */
    if (close(firmwareFd) == -1) {
        std::cerr << "Failed to close the FPGA Manager firmware: "
                  << FpgaManagerFirmware << '\n';
        return false;
    }

    return true;
}

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */
