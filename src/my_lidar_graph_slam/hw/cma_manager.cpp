
/* cma_manager.cpp */

#include "my_lidar_graph_slam/hw/cma_manager.hpp"

#include <iostream>
#include <dlfcn.h>

namespace MyLidarGraphSlam {
namespace Hardware {

/* Load the library function and return the pointer */
template <typename T>
T DlSym(void* handlePtr, const char* symbolName)
{
    return reinterpret_cast<T>(dlsym(handlePtr, symbolName));
}

/*
 * CMAMemoryManager class implementation
 */

/* Constructor */
CMAMemoryManager::CMAMemoryManager() :
    mLibPath(nullptr),
    mHandle(nullptr),
    mIsLoaded(false)
{
}

/* Destructor */
CMAMemoryManager::~CMAMemoryManager()
{
    /* Unload the CMA library if necessary */
    this->Unload();
}

/* Get the CMAMemoryManager instance */
CMAMemoryManager* CMAMemoryManager::Instance()
{
    static CMAMemoryManager theInstance;
    return &theInstance;
}

/* Load the CMA library (libcma) */
bool CMAMemoryManager::Load(const char* libPath)
{
    /* Return if the library is already loaded */
    if (this->mIsLoaded)
        return true;

    /* Set the path to the shared object library */
    this->mLibPath = libPath;

    /* Open the shared object library */
    this->mHandle = dlopen(this->mLibPath, RTLD_LAZY);

    if (this->mHandle == nullptr) {
        std::cerr << "dlopen() failed: "
                  << dlerror() << std::endl;
        return false;
    }

    /* Load the symbols in the shared object library */
    this->mCmaMmap = DlSym<CmaMmapType>(this->mHandle, "cma_mmap");
    if (this->mCmaMmap == nullptr)
        goto Fail;

    this->mCmaMunmap = DlSym<CmaMunmapType>(this->mHandle, "cma_munmap");
    if (this->mCmaMunmap == nullptr)
        goto Fail;

    this->mCmaAlloc = DlSym<CmaAllocType>(this->mHandle, "cma_alloc");
    if (this->mCmaAlloc == nullptr)
        goto Fail;

    this->mCmaGetPhyAddr = DlSym<CmaGetPhyAddrType>(
        this->mHandle, "cma_get_phy_addr");
    if (this->mCmaGetPhyAddr == nullptr)
        goto Fail;

    this->mCmaFree = DlSym<CmaFreeType>(this->mHandle, "cma_free");
    if (this->mCmaFree == nullptr)
        goto Fail;

    this->mCmaFlushCache = DlSym<CmaFlushCacheType>(
        this->mHandle, "cma_flush_cache");
    if (this->mCmaFlushCache == nullptr)
        goto Fail;

    this->mCmaInvalidateCache = DlSym<CmaInvalidateCacheType>(
        this->mHandle, "cma_invalidate_cache");
    if (this->mCmaInvalidateCache == nullptr)
        goto Fail;

    this->mXlnkReset = DlSym<XlnkResetType>(this->mHandle, "_xlnk_reset");
    if (this->mXlnkReset == nullptr)
        goto Fail;

    /* The library is successfully loaded */
    this->mIsLoaded = true;

    std::cerr << "Shared object library "
              << this->mLibPath
              << " is successfully loaded" << std::endl;
    return true;

Fail:
    std::cerr << "dlsym() failed: "
              << dlerror() << std::endl;
    return false;
}

/* Unload the CMA library (libcma) */
void CMAMemoryManager::Unload()
{
    if (this->mHandle == nullptr)
        return;

    if (dlclose(this->mHandle) != 0) {
        std::cerr << "dlclose() failed: "
                  << dlerror() << std::endl;
    }

    /* The library is unloaded */
    this->mIsLoaded = false;
}

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */
