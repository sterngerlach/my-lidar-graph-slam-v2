
# FindARM.cmake
# This file is taken from PyTorch repository (Thank you)
# https://github.com/pytorch/pytorch/blob/master/cmake/Modules/FindARM.cmake
# https://github.com/pytorch/pytorch/blob/master/CMakeLists.txt

if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    exec_program(cat ARGS "/proc/cpuinfo" OUTPUT_VARIABLE CPUINFO)

    # Neon instruction can be found on the majority part of modern ARM processor
    string(REGEX REPLACE "^.*(neon).*$" "\\1" NEON_THERE ${CPUINFO})
    string(COMPARE EQUAL "neon" "${NEON_THERE}" NEON_TRUE)

    if (NEON_TRUE)
        set(NEON_FOUND true CACHE BOOL "NEON available on host")
    else (NEON_TRUE)
        set(NEON_FOUND false CACHE BOOL "NEON available on host")
    endif (NEON_TRUE)

    # On ARMv8, neon is inherit and instead listed as 'asimd' in /proc/cpuinfo
    string(REGEX REPLACE "^.*(asimd).*$" "\\1" ASIMD_THERE ${CPUINFO})
    string(COMPARE EQUAL "asimd" "${ASIMD_THERE}" ASIMD_TRUE)

    if (ASIMD_TRUE)
        set(ASIMD_FOUND true CACHE BOOL "ASIMD/NEON available on host")
    else (ASIMD_TRUE)
        set(ASIMD_FOUND false CACHE BOOL "ASIMD/NEON available on host")
    endif (ASIMD_TRUE)

    # Find the processor type (for now OMAP3 or OMAP4)
    string(REGEX REPLACE "^.*(OMAP3).*$" "\\1" OMAP3_THERE ${CPUINFO})
    string(COMPARE EQUAL "OMAP3" "${OMAP3_THERE}" OMAP3_TRUE)

    if (OMAP3_TRUE)
        set(CORTEXA8_FOUND true CACHE BOOL "OMAP3 available on host")
    else (OMAP3_TRUE)
        set(CORTEXA8_FOUND false CACHE BOOL "OMAP3 available on host")
    endif (OMAP3_TRUE)

    # Find the processor type (for now OMAP3 or OMAP4)
    string(REGEX REPLACE "^.*(OMAP4).*$" "\\1" OMAP4_THERE ${CPUINFO})
    string(COMPARE EQUAL "OMAP4" "${OMAP4_THERE}" OMAP4_TRUE)

    if (OMAP4_TRUE)
        set(CORTEXA9_FOUND true CACHE BOOL "OMAP4 available on host")
    else (OMAP4_TRUE)
        set(CORTEXA9_FOUND false CACHE BOOL "OMAP4 available on host")
    endif (OMAP4_TRUE)
endif()

if (NOT NEON_FOUND)
    message(STATUS "Neon intrinsics not found on this machine.")
endif (NOT NEON_FOUND)

if (NOT ASIMD_FOUND)
    message(STATUS "Asimd/Neon not found on this machine.")
endif (NOT ASIMD_FOUND)

if (NOT CORTEXA8_FOUND)
    message(STATUS "OMAP3 processor not found on this machine.")
endif (NOT CORTEXA8_FOUND)

if (NOT CORTEXA9_FOUND)
    message(STATUS "OMAP4 processor not found on this machine.")
endif (NOT CORTEXA9_FOUND)
