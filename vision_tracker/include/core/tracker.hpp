#pragma once
#include <opencv2/opencv.hpp>
#include "core/kalman_filter.hpp"
#include "core/detector.hpp"   // 新增

enum class TrackState {
    TRACKING,
    LOST,
    SEARCHING
};

class TargetTracker {
public:
    TargetTracker();
    void update(const Detection& det, double dt);
    void updateLost(double dt);
    cv::Point2f getTargetPosition();
    TrackState getState() const { return state_; }
    void reset();

private:
    KalmanFilter kf_;
    TrackState state_;
    int lost_count_;
    int max_lost_frames_;
    cv::Point2f last_valid_pos_;
    cv::Point2f search_offset_;
    float search_angle_;
    
    cv::Point2f searchPattern(double dt);
};