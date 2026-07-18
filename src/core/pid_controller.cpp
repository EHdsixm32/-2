#include "core/pid_controller.hpp"
#include <algorithm>  // for std::clamp

PIDController::PIDController(const PIDParams& params) : params_(params) {}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}

double PIDController::update(double error, double dt) {
    if (dt < 1e-6) dt = 1e-3;
    double p_term = params_.kp * error;
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -params_.integral_limit, params_.integral_limit);
    double i_term = params_.ki * integral_;
    double derivative = (error - prev_error_) / dt;
    double d_term = params_.kd * derivative;
    double output = p_term + i_term + d_term;
    output = std::clamp(output, -params_.output_limit, params_.output_limit);
    prev_error_ = error;
    return output;
}