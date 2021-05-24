
/* mmio_shared.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_MMIO_SHARED_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_MMIO_SHARED_HPP

#include "my_lidar_graph_slam/hw/mmio.hpp"

#include <cstdint>
#include <memory>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * Utility class for shared memory mapped I/O
 */
class SharedMemoryMappedIO final
{
public:
    /* Constructor */
    SharedMemoryMappedIO(std::uint32_t baseAddress,
                         std::uint32_t addressRange);

    /* Destructor */
    ~SharedMemoryMappedIO() = default;

    /* Copy constructor */
    SharedMemoryMappedIO(const SharedMemoryMappedIO&) = default;
    /* Copy assignment operator */
    SharedMemoryMappedIO& operator=(const SharedMemoryMappedIO&) = default;

    /* Move constructor */
    SharedMemoryMappedIO(SharedMemoryMappedIO&&) = default;
    /* Move assignment operator */
    SharedMemoryMappedIO& operator=(SharedMemoryMappedIO&&) = default;

    /* Construct from an existing memory mapped object */
    SharedMemoryMappedIO(MemoryMappedIO&& other) noexcept;
    /* Assign an existing memory mapped object */
    SharedMemoryMappedIO& operator=(MemoryMappedIO&& other) noexcept;

    /* Construct from an existing shared memory mapped object */
    SharedMemoryMappedIO(
        std::shared_ptr<MemoryMappedIO>&& other) noexcept;
    /* Assign an existing shared memory mapped object */
    SharedMemoryMappedIO& operator=(
        std::shared_ptr<MemoryMappedIO>&& other) noexcept;

    /* Construct from an existing memory mapped object */
    SharedMemoryMappedIO(
        std::unique_ptr<MemoryMappedIO>&& other) noexcept;
    /* Assign an existing memory mapped object */
    SharedMemoryMappedIO& operator=(
        std::unique_ptr<MemoryMappedIO>&& other) noexcept;

    /* Wrapper methods */
    /* Read from the memory mapped device */
    inline std::uint32_t Read(std::uint32_t offsetInBytes) const
    { return this->mImpl->Read(offsetInBytes); }

    /* Write to the memory mapped device */
    inline void Write(std::uint32_t offsetInBytes, std::uint32_t value) const
    { return this->mImpl->Write(offsetInBytes, value); }

    /* Get the raw pointer to the device register */
    inline volatile std::uint32_t* Ptr(std::uint32_t offsetInBytes) const
    { return this->mImpl->Ptr(offsetInBytes); }

    /* Get the number of bytes that user requested */
    inline std::uint32_t AddressRange() const noexcept
    { return this->mImpl->AddressRange(); }

    /* Get the actual number of bytes that is a multiple of the page size */
    inline std::uint32_t MappedAddressRange() const noexcept
    { return this->mImpl->MappedAddressRange(); }

    /* Get the offset relative to the start of the mapped area */
    inline std::uint32_t AddressOffset() const noexcept
    { return this->mImpl->AddressOffset(); }

private:
    /* Shared pointer to the implementation */
    std::shared_ptr<MemoryMappedIO> mImpl;
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_MMIO_SHARED_HPP */
