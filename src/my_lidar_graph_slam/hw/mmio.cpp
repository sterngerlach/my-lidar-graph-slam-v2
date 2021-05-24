
/* mmio.cpp */

#include "my_lidar_graph_slam/hw/mmio.hpp"
#include "my_lidar_graph_slam/util.hpp"

#include <sstream>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * MemoryMappedIO class implementations
 */

/* Constructor */
MemoryMappedIO::MemoryMappedIO(std::uint32_t baseAddress,
                               std::uint32_t addressRange) :
    mBaseAddress(0),
    mAddressOffset(0),
    mMappedAddressRange(0),
    mMappedPtr(nullptr)
{
    /* Check the base address */
    XAssert(baseAddress % 4 == 0, "Base address must be 4-byte aligned");
    /* Check the address range */
    XAssert(addressRange % 4 == 0, "Address range must be multiple of 4");
    /* Check the root permission */
    XAssert(geteuid() == 0, "Root permission is required");

    /* Open /dev/mem for memory access */
    int fd;

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
        std::stringstream errorMsg;
        errorMsg << "open() failed: " << strerror(errno) << std::endl;
        const std::string errorMsgStr = errorMsg.str();
        XAssert(false, errorMsgStr.c_str());
    }

    /* Get the page size of the system */
    const std::uint32_t pageSize =
        static_cast<std::uint32_t>(sysconf(_SC_PAGE_SIZE));
    /* Align the base address with the page */
    const std::uint32_t virtualBaseAddr = baseAddress & ~(pageSize - 1);
    /* Calculate the address offset */
    const std::uint32_t virtualOffset = baseAddress - virtualBaseAddr;
    /* Calculate the mapping length */
    const std::uint32_t mappedLength = virtualOffset + addressRange;

    /* Memory map the device */
    char* mappedPtr = reinterpret_cast<char*>(
        mmap(nullptr, static_cast<std::size_t>(mappedLength),
             PROT_READ | PROT_WRITE, MAP_SHARED,
             fd, static_cast<off_t>(virtualBaseAddr)));

    if (mappedPtr == MAP_FAILED) {
        std::stringstream errorMsg;
        errorMsg << "mmap() failed: " << strerror(errno) << std::endl;
        const std::string errorMsgStr = errorMsg.str();
        XAssert(false, errorMsgStr.c_str());
    }

    /* Close /dev/mem */
    if (close(fd) == -1) {
        std::stringstream errorMsg;
        errorMsg << "close() failed: " << strerror(errno) << std::endl;
        const std::string errorMsgStr = errorMsg.str();
        XAssert(false, errorMsgStr.c_str());
    }

    /* Create memory mapped I/O instance */
    this->mBaseAddress = virtualBaseAddr;
    this->mAddressOffset = virtualOffset;
    this->mMappedAddressRange = mappedLength;
    this->mMappedPtr = mappedPtr;
}

/* Destructor */
MemoryMappedIO::~MemoryMappedIO()
{
    /* Unmap the device if necessary */
    this->Unmap();
}

/* Move constructor */
MemoryMappedIO::MemoryMappedIO(MemoryMappedIO&& other) noexcept :
    mBaseAddress(0),
    mAddressOffset(0),
    mMappedAddressRange(0),
    mMappedPtr(nullptr)
{
    /* Copy the information from the source object */
    this->mBaseAddress = other.mBaseAddress;
    this->mAddressOffset = other.mAddressOffset;
    this->mMappedAddressRange = other.mMappedAddressRange;
    this->mMappedPtr = other.mMappedPtr;

    /* Clear the information of the source object so that
     * the pointer is not unmapped for multiple times */
    other.mBaseAddress = 0;
    other.mAddressOffset = 0;
    other.mMappedAddressRange = 0;
    other.mMappedPtr = nullptr;
}

/* Move assignment operator */
MemoryMappedIO& MemoryMappedIO::operator=(MemoryMappedIO&& other) noexcept
{
    if (this == &other)
        return *this;

    /* Unmap the current pointer first */
    this->Unmap();

    /* Copy the information from the source object */
    this->mBaseAddress = other.mBaseAddress;
    this->mAddressOffset = other.mAddressOffset;
    this->mMappedAddressRange = other.mMappedAddressRange;
    this->mMappedPtr = other.mMappedPtr;

    /* Clear the information of the source object so that
     * the pointer is not unmapped for multiple times */
    other.mBaseAddress = 0;
    other.mAddressOffset = 0;
    other.mMappedAddressRange = 0;
    other.mMappedPtr = nullptr;

    return *this;
}

/* Read from the memory mapped device */
std::uint32_t MemoryMappedIO::Read(
    std::uint32_t offsetInBytes) const
{
    /* Get the raw pointer */
    volatile std::uint32_t* regPtr = this->Ptr(offsetInBytes);
    /* Read the device */
    return *regPtr;
}

/* Write to the memory mapped device */
void MemoryMappedIO::Write(
    std::uint32_t offsetInBytes,
    std::uint32_t value) const
{
    /* Get the raw pointer */
    volatile std::uint32_t* regPtr = this->Ptr(offsetInBytes);
    /* Write to the device */
    *regPtr = value;
}

/* Get the raw pointer to the device register */
volatile std::uint32_t* MemoryMappedIO::Ptr(
    std::uint32_t offsetInBytes) const
{
    XAssert(offsetInBytes <= this->AddressRange() - 4,
            "Address offset is out of range");
    XAssert(offsetInBytes % 4 == 0,
            "Offset must be 4-byte aligned");
    XAssert(this->mMappedPtr != nullptr,
            "Invalid pointer to the mapped area");

    volatile std::uint32_t* regPtr =
        reinterpret_cast<volatile std::uint32_t*>(
            this->mMappedPtr + this->mAddressOffset + offsetInBytes);

    return regPtr;
}

/* Unmap the memory mapped device */
void MemoryMappedIO::Unmap()
{
    if (this->mMappedPtr == nullptr)
        return;

    /* Unmap the device */
    if (munmap(this->mMappedPtr,
               static_cast<std::size_t>(this->mMappedAddressRange)) == -1)
        std::cerr << "munmap() failed: " << strerror(errno) << std::endl;

    /* Clear the instance */
    this->mBaseAddress = 0;
    this->mAddressOffset = 0;
    this->mMappedAddressRange = 0;
    this->mMappedPtr = nullptr;
}

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */
