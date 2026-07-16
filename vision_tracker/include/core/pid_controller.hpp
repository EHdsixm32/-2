#pragma once
#include <algorithm>
#include <cmath>

struct PIDParams {
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_limit = 10.0;
    double output_limit = 100.0;
};

class PIDController {
public:
    PIDController(const PIDParams& params);
    void reset();
    double update(double error, double dt);

private:
    PIDParams params_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
};