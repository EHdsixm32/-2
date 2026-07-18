#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <GxIAPI.h>

class Camera {
public:
    Camera(int device_id, int width, int height);
    ~Camera();

    bool open();
    bool read(cv::Mat& frame);
    void setFocalLength(float fx) { focal_length_px_ = fx; }
    float getFocalLength() const { return focal_length_px_; }
    void close();

private:
    GX_DEV_HANDLE device_handle_ = nullptr;
    int device_id_;
    int width_;
    int height_;
    float focal_length_px_ = 600.0;

    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat latest_frame_;
    std::atomic<bool> frame_updated_{false};
    std::atomic<bool> is_streaming_{false};

    // 新 SDK 回调签名：GX_FRAME_CALLBACK_PARAM*
    static void GX_STDC OnImageCallback(GX_FRAME_CALLBACK_PARAM* pFrame);
    void processRawImage(GX_FRAME_CALLBACK_PARAM* pFrame);
};