cmake_minimum_required(VERSION 3.16.3)

project(sp_xz_vision)

add_definitions(-DSPDLOG_FMT_EXTERNAL=1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(function/auto_buff)
add_subdirectory(tools)

# #################tests##################
add_executable(auto_buff_test test/auto_buff_test.cpp)

target_link_libraries(auto_buff_test ${OpenCV_LIBS} auto_buff yaml-cpp tools fmt::fmt)
