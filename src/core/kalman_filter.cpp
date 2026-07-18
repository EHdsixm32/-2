#include "core/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {
    // 状态转移矩阵: x_{k+1} = F * x_k
    F_ = Eigen::Matrix<float, 4, 4>::Identity();
    F_(0, 2) = 1.0; F_(1, 3) = 1.0; // dt默认为1，由外部时间步长控制，这里假设匀速

    // 观测矩阵: z_k = H * x_k
    H_ = Eigen::Matrix<float, 2, 4>::Zero();
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;

    // 过程噪声 (调参重点)
    Q_ = Eigen::Matrix<float, 4, 4>::Identity() * 0.05;

    // 观测噪声 (调参重点)
    R_ = Eigen::Matrix<float, 2, 2>::Identity() * 10.0;

    // 初始化协方差
    P_ = Eigen::Matrix<float, 4, 4>::Identity() * 100.0;
}

void KalmanFilter::init(const cv::Point2f& pos) {
    X_ << pos.x, pos.y, 0, 0;
    P_ = Eigen::Matrix<float, 4, 4>::Identity() * 100.0;
    initialized_ = true;
}

cv::Point2f KalmanFilter::predict() {
    if (!initialized_) return cv::Point2f(0, 0);
    X_ = F_ * X_;
    P_ = F_ * P_ * F_.transpose() + Q_;
    return cv::Point2f(X_(0), X_(1));
}

cv::Point2f KalmanFilter::update(const cv::Point2f& measurement) {
    if (!initialized_) return cv::Point2f(0, 0);
    
    Eigen::Matrix<float, 2, 1> Z;
    Z << measurement.x, measurement.y;

    // 卡尔曼增益
    Eigen::Matrix<float, 2, 2> S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<float, 4, 2> K = P_ * H_.transpose() * S.inverse();

    // 更新状态
    X_ = X_ + K * (Z - H_ * X_);
    P_ = (Eigen::Matrix<float, 4, 4>::Identity() - K * H_) * P_;

    return cv::Point2f(X_(0), X_(1));
}