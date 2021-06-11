
# CMakeLists.txt

cmake_minimum_required(VERSION 3.8)

project(my_lidar_graph_slam CXX)

# Find libpng
find_package(PNG REQUIRED)

# Find Boost package
set(BOOST_USE_STATIC_LIBS ON)
find_package(Boost 1.65.1 REQUIRED system timer)

# Find Eigen package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find Neon intrinsics
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
find_package(ARM)

set(CMAKE_DEBUG_POSTFIX "d")

add_library(lidar_graph_slam)

add_subdirectory(include)
add_subdirectory(src)

target_include_directories(
    lidar_graph_slam PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(lidar_graph_slam PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(lidar_graph_slam PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(lidar_graph_slam PUBLIC
    ${g2o_INCLUDE_DIR} /usr/include/suitesparse)
target_link_libraries(lidar_graph_slam PUBLIC m rt pthread dl)
target_link_libraries(lidar_graph_slam PUBLIC Eigen3::Eigen)
target_link_libraries(lidar_graph_slam PUBLIC PNG::PNG)
target_link_libraries(lidar_graph_slam PUBLIC ${Boost_LIBRARIES})
target_link_libraries(lidar_graph_slam PUBLIC
    cholmod g2o_core g2o_stuff g2o_solver_cholmod g2o_types_slam2d)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.1)
        target_link_libraries(lidar_graph_slam PUBLIC stdc++fs)
    endif()
endif()

target_compile_options(lidar_graph_slam PUBLIC
    $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wno-psabi>
    $<$<CONFIG:Release>:-O3>
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:Debug>>:-O0 -ggdb>)
target_compile_features(lidar_graph_slam PUBLIC cxx_std_17)
set_target_properties(lidar_graph_slam PROPERTIES
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

# if (NEON_TRUE)
#     message(STATUS "Neon found with compiler flag: -mfpu=neon -D__NEON__")
#     target_compile_options(lidar_graph_slam PUBLIC -mfpu=neon -D__NEON__)
# endif (NEON_TRUE)

add_executable(slam_launch
    ${PROJECT_SOURCE_DIR}/src/my_lidar_graph_slam/slam_launcher.cpp)
target_link_libraries(slam_launch lidar_graph_slam)
set_target_properties(slam_launch PROPERTIES
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
