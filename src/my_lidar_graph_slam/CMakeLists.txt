
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/util.cpp)

add_subdirectory(grid_map)
add_subdirectory(grid_map_new)
add_subdirectory(hw)
add_subdirectory(io)
add_subdirectory(mapping)
add_subdirectory(metric)
