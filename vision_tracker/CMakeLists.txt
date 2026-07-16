cmake_minimum_required(VERSION 3.10)
project(vision_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui dnn)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)   # 新增

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(vision_tracker ${SOURCES})

target_link_libraries(vision_tracker 
    ${OpenCV_LIBS} 
    ${EIGEN3_LIBRARIES} 
    ${YAML_CPP_LIBRARIES} 
    ${SPDLOG_LIBRARIES}
    fmt::fmt                # 新增
    pthread
)

target_compile_options(vision_tracker PRIVATE -O3 -march=native)
