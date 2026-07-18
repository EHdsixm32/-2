#include "core/tracker.hpp"
#include <cmath>

TargetTracker::TargetTracker() 
    : state_(TrackState::LOST), lost_count_(0), max_lost_frames_(15), 
      search_angle_(0), search_offset_(0, 0) {}

void TargetTracker::update(const Detection& det, double dt) {
    cv::Point2f measurement = det.center;
    if (!kf_.isInitialized()) {
        kf_.init(measurement);
        last_valid_pos_ = measurement;
        state_ = TrackState::TRACKING;
        lost_count_ = 0;
        return;
    }

    // 防止跳变：检查与预测位置的距离
    cv::Point2f pred = kf_.predict();
    float dist = cv::norm(pred - measurement);
    if (dist > 150.0) { 
        // 跳变太大，认为是误检，不更新
        lost_count_++;
        if (lost_count_ > max_lost_frames_) state_ = TrackState::LOST;
        return;
    }

    // 正常更新
    kf_.update(measurement);
    last_valid_pos_ = measurement;
    lost_count_ = 0;
    state_ = TrackState::TRACKING;
}

void TargetTracker::updateLost(double dt) {
    if (state_ == TrackState::TRACKING) {
        lost_count_++;
        if (lost_count_ > max_lost_frames_) {
            state_ = TrackState::LOST;
            search_angle_ = 0;
        }
    } else if (state_ == TrackState::LOST) {
        // 尝试搜索
        state_ = TrackState::SEARCHING;
    } else if (state_ == TrackState::SEARCHING) {
        // 保持搜索，如果检测到目标会在update中切回TRACKING
    }
}

cv::Point2f TargetTracker::getTargetPosition() {
    if (state_ == TrackState::TRACKING) {
        if (kf_.isInitialized()) {
            return kf_.predict(); // 返回平滑预测值
        }
        return last_valid_pos_;
    } else if (state_ == TrackState::SEARCHING) {
        // 螺旋搜索逻辑：以最后已知位置为中心
        search_angle_ += 0.15;
        float radius = 20.0 + 10.0 * search_angle_;
        float x = last_valid_pos_.x + radius * cos(search_angle_);
        float y = last_valid_pos_.y + radius * sin(search_angle_);
        return cv::Point2f(x, y);
    }
    return cv::Point2f(-1, -1); // LOST状态无效
}

void TargetTracker::reset() {
    state_ = TrackState::LOST;
    lost_count_ = 0;
    search_angle_ = 0;
}