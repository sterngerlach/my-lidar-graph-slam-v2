
/* cma_manager.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_CMA_MANAGER_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_CMA_MANAGER_HPP

#include <cstdint>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * Wrapper class for CMA memory management
 * SDSoC kernel driver (/dev/xlnk), SDSoC library (sds_lib), and
 * helper library provided by Pynq (libcma) is required for this class to work
 */
class CMAMemoryManager
{
private:
    /* Type definitions */
    /* unsigned long cma_mmap(uint32_t phyAddr, uint32_t len); */
    using CmaMmapType = unsigned long(*)(std::uint32_t, std::uint32_t);

    /* unsigned long cma_munmap(void *buf, uint32_t len); */
    using CmaMunmapType = unsigned long(*)(void*, std::uint32_t);

    /* void *cma_alloc(uint32_t len, uint32_t cacheable); */
    using CmaAllocType = void*(*)(std::uint32_t, std::uint32_t);

    /* unsigned long cma_get_phy_addr(void *buf); */
    using CmaGetPhyAddrType = unsigned long(*)(void*);

    /* void cma_free(void *buf); */
    using CmaFreeType = void(*)(void*);

    /* uint32_t cma_pages_available(); */
    using CmaPagesAvailableType = std::uint32_t(*)();

    /* void cma_flush_cache(void* buf, unsigned int phyAddr, int size); */
    using CmaFlushCacheType = void(*)(void*, unsigned int, int);

    /* void cma_invalidate_cache(void* buf, unsigned int phyAddr, int size); */
    using CmaInvalidateCacheType = void(*)(void*, unsigned int, int);

    /* void _xlnk_reset(); */
    using XlnkResetType = void(*)();

private:
    /* Default constructor */
    CMAMemoryManager();
    /* Destructor */
    ~CMAMemoryManager();

public:
    /* Copy constructor (disabled) */
    CMAMemoryManager(const CMAMemoryManager&) = delete;
    /* Move constructor (disabled) */
    CMAMemoryManager(CMAMemoryManager&&) = delete;
    /* Copy assignment operator (disabled) */
    CMAMemoryManager& operator=(const CMAMemoryManager&) = delete;
    /* Move assignment operator (disabled) */
    CMAMemoryManager& operator=(CMAMemoryManager&&) = delete;

    /* Get the CMAMemoryManager instance */
    static CMAMemoryManager* Instance();

    /* Check if the library is loaded */
    inline bool IsLoaded() const { return this->mIsLoaded; }

    /* Load the CMA library (libcma) */
    bool Load(const char* libPath = DefaultLibPath);

    /* Unload the CMA library (libcma) */
    void Unload();

    /* Library functions */
    inline void* CmaAllocate(std::uint32_t len,
                             std::uint32_t isCacheable = 0U) const
    { return this->mCmaAlloc(len, isCacheable); }

    inline unsigned long CmaGetPhysicalAddress(void* buf) const
    { return this->mCmaGetPhyAddr(buf); }

    inline void CmaFree(void* buf) const
    { this->mCmaFree(buf); }

    inline std::uint32_t CmaNumOfPagesAvailable() const
    { return this->mCmaPagesAvailable(); }

private:
    /* Path to the CMA library */
    const char*            mLibPath;
    /* Handle for the CMA library (shared object) */
    void*                  mHandle;
    /* Flag to determine whether the library is loaded */
    bool                   mIsLoaded;

    /* Function pointers */
    CmaMmapType            mCmaMmap;
    CmaMunmapType          mCmaMunmap;
    CmaAllocType           mCmaAlloc;
    CmaGetPhyAddrType      mCmaGetPhyAddr;
    CmaFreeType            mCmaFree;
    CmaPagesAvailableType  mCmaPagesAvailable;
    CmaFlushCacheType      mCmaFlushCache;
    CmaInvalidateCacheType mCmaInvalidateCache;
    XlnkResetType          mXlnkReset;

    /* Default path to the CMA helper library */
    static constexpr const char* DefaultLibPath = "/usr/lib/libcma.so";
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_CMA_MANAGER_HPP */
