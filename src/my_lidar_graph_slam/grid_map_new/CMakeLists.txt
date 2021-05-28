
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_binary_bayes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_values.cpp)
