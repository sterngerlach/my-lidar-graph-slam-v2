
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/const_grid_cell.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/discrete_grid_cell.cpp)
