
/* mmio_shared.cpp */

#include "my_lidar_graph_slam/hw/mmio_shared.hpp"

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * SharedMemoryMappedIO class implementations
 */

/* Constructor */
SharedMemoryMappedIO::SharedMemoryMappedIO(
    std::uint32_t baseAddress,
    std::uint32_t addressRange) :
    mImpl(std::make_shared<MemoryMappedIO>(baseAddress, addressRange))
{
}

/* Construct from an existing memory mapped object */
SharedMemoryMappedIO::SharedMemoryMappedIO(
    MemoryMappedIO&& other) noexcept :
    mImpl(std::make_shared<MemoryMappedIO>(std::move(other)))
{
}

/* Assign an existing memory mapped object */
SharedMemoryMappedIO& SharedMemoryMappedIO::operator=(
    MemoryMappedIO&& other) noexcept
{
    this->mImpl = std::make_shared<MemoryMappedIO>(std::move(other));
    return *this;
}

/* Construct from an existing shared memory mapped object */
SharedMemoryMappedIO::SharedMemoryMappedIO(
    std::shared_ptr<MemoryMappedIO>&& other) noexcept :
    mImpl(std::move(other))
{
}

/* Assign an existing shared memory mapped object */
SharedMemoryMappedIO& SharedMemoryMappedIO::operator=(
    std::shared_ptr<MemoryMappedIO>&& other) noexcept
{
    this->mImpl = std::move(other);
    return *this;
}

/* Construct from an existing memory mapped object */
SharedMemoryMappedIO::SharedMemoryMappedIO(
    std::unique_ptr<MemoryMappedIO>&& other) noexcept :
    mImpl(std::move(other))
{
}

/* Assign an existing memory mapped object */
SharedMemoryMappedIO& SharedMemoryMappedIO::operator=(
    std::unique_ptr<MemoryMappedIO>&& other) noexcept
{
    this->mImpl = std::move(other);
    return *this;
}

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */
