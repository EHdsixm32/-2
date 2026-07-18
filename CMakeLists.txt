cmake_minimum_required(VERSION 3.10)
project(vision_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 依赖包
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui dnn)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# 大恒 SDK 路径
set(DAHENG_BASE "/home/ehdsixm/spvision/Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.4.2507.9231/Galaxy_camera")
set(DAHENG_INC_DIR "${DAHENG_BASE}/inc")
set(DAHENG_LIB_DIR "${DAHENG_BASE}/lib/x86_64")

set(DAHENG_LIBS
    "${DAHENG_LIB_DIR}/libgxiapi.so"
    "${DAHENG_LIB_DIR}/liblog4cplus_gx.so"
)

# ONNXRuntime（手动安装）
find_library(ONNXRUNTIME_LIB NAMES onnxruntime PATHS /usr/local/lib)
if(NOT ONNXRUNTIME_LIB)
    message(WARNING "ONNXRuntime not found, using OpenCV DNN fallback")
endif()

# 包含目录
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${DAHENG_INC_DIR})
include_directories(/usr/local/include)  # ONNXRuntime 头文件

# 源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 可执行文件
add_executable(vision_tracker ${SOURCES})

# 链接库（必须在 add_executable 之后）
target_link_libraries(vision_tracker
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${SPDLOG_LIBRARIES}
    fmt::fmt
    ${DAHENG_LIBS}
    ${ONNXRUNTIME_LIB}      # 若未找到则为空
    pthread
)

target_compile_options(vision_tracker PRIVATE -O0 -g)