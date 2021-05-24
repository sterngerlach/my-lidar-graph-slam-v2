
/* cma_memory.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_CMA_MEMORY_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_CMA_MEMORY_HPP

#include <cstdint>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * RAII class for CMA memory
 */
class CMAMemory final
{
public:
    /* Default constructor */
    CMAMemory();
    /* Constructor */
    CMAMemory(std::uint32_t lengthInBytes);
    /* Destructor */
    ~CMAMemory();

    /* Copy constructor (disabled) */
    CMAMemory(const CMAMemory&) = delete;
    /* Copy assignment operator (disabled) */
    CMAMemory& operator=(const CMAMemory&) = delete;

    /* Move constructor */
    CMAMemory(CMAMemory&& other) noexcept;
    /* Move assignment operator */
    CMAMemory& operator=(CMAMemory&& other) noexcept;

    /* Initialize the CMA memory */
    void Initialize(std::uint32_t lengthInBytes);

    /* Get the pointer to the CMA memory */
    template <typename T>
    inline T* Ptr() { return static_cast<T*>(this->mPtr); }
    /* Get the pointer to the CMA memory */
    template <typename T>
    inline const T* Ptr() const { return static_cast<T*>(this->mPtr); }
    /* Get the size of the CMA memory */
    inline std::uint32_t Size() const { return this->mSize; }
    /* Get the physical address of the CMA memory */
    inline std::uint32_t PhysicalAddress() const
    { return this->mPhysicalAddress; }

private:
    /* Pointer to the CMA memory */
    void*         mPtr;
    /* Address range (size) of the CMA memory */
    std::uint32_t mSize;
    /* Physical address of the CMA memory */
    std::uint32_t mPhysicalAddress;
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_CMA_MEMORY_HPP */
