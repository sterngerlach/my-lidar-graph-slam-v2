
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bresenham.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cost_function_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fpga_module_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_detector_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loss_function_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/memory_usage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph_optimizer_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_filter_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/score_function_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/slam_module_factory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/util.cpp)

add_subdirectory(grid_map)
add_subdirectory(grid_map_new)
add_subdirectory(hw)
add_subdirectory(io)
add_subdirectory(mapping)
add_subdirectory(metric)
add_subdirectory(network)
