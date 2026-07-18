#include "hardware/camera.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <cstring>
#include <GxIAPI.h>

static bool g_lib_initialized = false;

Camera::Camera(int device_id, int width, int height)
    : device_id_(device_id), width_(width), height_(height) {
    if (!g_lib_initialized) {
        GXInitLib();
        g_lib_initialized = true;
        spdlog::info("Daheng Galaxy SDK initialized.");
    }
}

Camera::~Camera() {
    close();
}

void GX_STDC Camera::OnImageCallback(GX_FRAME_CALLBACK_PARAM* pFrame) {
    if (!pFrame) return;
    Camera* self = static_cast<Camera*>(pFrame->pUserParam);
    if (self) self->processRawImage(pFrame);
}

void Camera::processRawImage(GX_FRAME_CALLBACK_PARAM* pFrame) {
    if (!pFrame || !pFrame->pImgBuf || pFrame->nImgSize == 0) return;
    int width = pFrame->nWidth;
    int height = pFrame->nHeight;
    // 大恒水星系列通常输出 Bayer 8bit，先尝试 GRBG，若颜色不对可改其他顺序
    cv::Mat raw_mat(height, width, CV_8UC1, const_cast<void*>(pFrame->pImgBuf));
    cv::Mat rgb_mat;
    cv::cvtColor(raw_mat, rgb_mat, cv::COLOR_BayerGR2BGR);
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = rgb_mat.clone();
        frame_updated_ = true;
    }
    frame_cv_.notify_one();
}

bool Camera::open() {
    if (is_streaming_) return true;
    if (device_handle_) {
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
    }

    // 1. 枚举设备
    uint32_t nDevNum = 0;
    GX_STATUS status = GXUpdateDeviceList(&nDevNum, 1000);
    if (status != GX_STATUS_SUCCESS || nDevNum == 0) {
        spdlog::error("No Daheng camera found. Check USB connection.");
        return false;
    }
    spdlog::info("Found {} camera(s).", nDevNum);

    // 2. 准备打开参数：通过索引 0 打开，独占模式
    GX_OPEN_PARAM openParam;
    memset(&openParam, 0, sizeof(GX_OPEN_PARAM));
    // 注意：pszContent 必须指向有效的字符串，索引从 1 开始，但我们用 "0" 表示第一个设备
    // 但实际上 GX_OPEN_INDEX 要求 pszContent 是索引的字符串表示，如 "1", "2" 等
    // 根据大恒示例，通常用 "0" 表示第一个设备，但更安全的是使用 SN 或直接填 NULL？
    // 经过查阅，若 openMode = GX_OPEN_INDEX，pszContent 应填入 "1"（第一个设备序号从1开始）
    // 我们填入 "1"
    const char* indexStr = "1";
    openParam.pszContent = const_cast<char*>(indexStr);
    openParam.openMode = GX_OPEN_INDEX;        // 3
    openParam.accessMode = GX_ACCESS_EXCLUSIVE; // 4

    // 3. 尝试多次打开（处理设备忙的情况）
    bool opened = false;
    for (int retry = 0; retry < 5; ++retry) {
        status = GXOpenDevice(&openParam, &device_handle_);
        if (status == GX_STATUS_SUCCESS) {
            opened = true;
            break;
        }
        spdlog::warn("Open attempt {} failed, error: 0x{:08X}, retrying...", retry+1, status);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!opened) {
        // 获取详细错误信息
        char errMsg[256];
        size_t size = sizeof(errMsg);
        GX_STATUS errCode;
        GXGetLastError(&errCode, errMsg, &size);
        spdlog::error("Failed to open camera after 5 attempts. Last error: 0x{:08X} - {}", errCode, errMsg);
        spdlog::error("Please ensure no other application (like GxViewer) is using the camera.");
        spdlog::error("You may also try to reboot your computer and run this program first.");
        return false;
    }
    spdlog::info("Camera opened via GX_OPEN_PARAM (index 1).");

    // 4. 配置相机参数
    GXSetEnum(device_handle_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GXSetEnum(device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GXSetInt(device_handle_, GX_INT_WIDTH, width_);
    GXSetInt(device_handle_, GX_INT_HEIGHT, height_);


    // ===== 调节亮度与白平衡 =====
    GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    GXSetEnum(device_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    GXSetEnum(device_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    // ==========================


    // 5. 注册回调
    status = GXRegisterCaptureCallback(device_handle_, this, OnImageCallback);
    if (status != GX_STATUS_SUCCESS) {
        spdlog::error("Failed to register callback, error: 0x{:08X}", status);
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
        return false;
    }

    // 6. 开始采集
    status = GXStreamOn(device_handle_);
    if (status != GX_STATUS_SUCCESS) {
        spdlog::error("Failed to start stream, error: 0x{:08X}", status);
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
        return false;
    }

    is_streaming_ = true;
    spdlog::info("Camera opened successfully: {}x{}", width_, height_);
    return true;
}

bool Camera::read(cv::Mat& frame) {
    if (!is_streaming_ || !device_handle_) return false;
    std::unique_lock<std::mutex> lock(frame_mutex_);
    if (!frame_cv_.wait_for(lock, std::chrono::milliseconds(2000), [this]{ return frame_updated_.load(); })) {
        spdlog::warn("Camera read timeout.");
        return false;
    }
    if (latest_frame_.empty()) return false;
    latest_frame_.copyTo(frame);
    frame_updated_ = false;
    return true;
}

void Camera::close() {
    if (device_handle_) {
        if (is_streaming_) {
            GXStreamOff(device_handle_);
            is_streaming_ = false;
        }
        GXUnregisterCaptureCallback(device_handle_);
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
        spdlog::info("Camera closed.");
    }
}