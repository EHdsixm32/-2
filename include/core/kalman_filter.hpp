#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class KalmanFilter {
public:
    KalmanFilter();
    void init(const cv::Point2f& pos);
    cv::Point2f predict();
    cv::Point2f update(const cv::Point2f& measurement);
    bool isInitialized() const { return initialized_; }

private:
    Eigen::Matrix<float, 4, 1> X_;  // [x, y, vx, vy]
    Eigen::Matrix<float, 4, 4> P_;
    Eigen::Matrix<float, 4, 4> F_;
    Eigen::Matrix<float, 2, 4> H_;
    Eigen::Matrix<float, 4, 4> Q_;
    Eigen::Matrix<float, 2, 2> R_;
    bool initialized_ = false;
};