
/* mmio.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_MMIO_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_MMIO_HPP

#include <cstdint>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * Utility class for memory mapped I/O
 */
class MemoryMappedIO final
{
public:
    /* Constructor */
    MemoryMappedIO(std::uint32_t baseAddress,
                   std::uint32_t addressRange);

    /* Destructor */
    ~MemoryMappedIO();

    /* Copy constructor (disabled) */
    MemoryMappedIO(const MemoryMappedIO&) = delete;
    /* Copy assignment operator (disabled) */
    MemoryMappedIO& operator=(const MemoryMappedIO&) = delete;

    /* Move constructor */
    MemoryMappedIO(MemoryMappedIO&&) noexcept;
    /* Move assignment operator */
    MemoryMappedIO& operator=(MemoryMappedIO&&) noexcept;

    /* Read from the memory mapped device */
    std::uint32_t Read(std::uint32_t offsetInBytes) const;

    /* Write to the memory mapped device */
    void Write(std::uint32_t offsetInBytes, std::uint32_t value) const;

    /* Get the raw pointer to the device register */
    volatile std::uint32_t* Ptr(std::uint32_t offsetInBytes) const;

    /* Get the number of bytes that user requested */
    inline std::uint32_t AddressRange() const noexcept
    { return this->mMappedAddressRange - this->mAddressOffset; }

    /* Get the actual number of bytes that is a multiple of the page size */
    inline std::uint32_t MappedAddressRange() const noexcept
    { return this->mMappedAddressRange; }

    /* Get the offset relative to the start of the mapped area */
    inline std::uint32_t AddressOffset() const noexcept
    { return this->mAddressOffset; }

private:
    /* Unmap the memory mapped device */
    void Unmap();

private:
    /* Base address (physical address) of the memory mapped I/O */
    std::uint32_t mBaseAddress;
    /* Address offset relative to the start of the mapped area */
    std::uint32_t mAddressOffset;
    /* Actual number of bytes that is a multiple of the page size */
    std::uint32_t mMappedAddressRange;
    /* Pointer to the start of the mapped area */
    char*         mMappedPtr;
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_MMIO_HPP */
