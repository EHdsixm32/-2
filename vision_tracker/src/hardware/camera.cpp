#include "hardware/camera.hpp"
#include <spdlog/spdlog.h>

Camera::Camera(int device_id, int width, int height)
    : device_id_(device_id), width_(width), height_(height) {}

bool Camera::open() {
    cap_.open(device_id_);
    if (!cap_.isOpened()) {
        spdlog::error("Camera {} open failed!", device_id_);
        return false;
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, 30);
    spdlog::info("Camera {} opened: {}x{}", device_id_, width_, height_);
    return true;
}

bool Camera::read(cv::Mat& frame) {
    if (!cap_.isOpened()) return false;
    cap_ >> frame;
    return !frame.empty();
}

Camera::~Camera() {
    if (cap_.isOpened()) cap_.release();
}