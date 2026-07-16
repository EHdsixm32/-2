#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class Camera {
public:
    Camera(int device_id, int width, int height);
    bool open();
    bool read(cv::Mat& frame);
    void setFocalLength(float fx) { focal_length_px_ = fx; }
    float getFocalLength() const { return focal_length_px_; }
    ~Camera();

private:
    cv::VideoCapture cap_;
    int device_id_;
    int width_;
    int height_;
    float focal_length_px_ = 600.0;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool loadCalibration();
};