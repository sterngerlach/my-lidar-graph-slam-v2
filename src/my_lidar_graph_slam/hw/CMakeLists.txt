
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/axi_dma.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axi_gpio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bitstream_loader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cma_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cma_memory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mmio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mmio_shared.cpp)
