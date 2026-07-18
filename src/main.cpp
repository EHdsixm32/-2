#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "core/detector.hpp"
#include "core/tracker.hpp"
#include "core/pid_controller.hpp"
#include "hardware/camera.hpp"
#include "hardware/serial_comm.hpp"
#include "utils/config_manager.hpp"
#include "utils/logger.hpp"

int main(int argc, char** argv) {
    Logger::init();
    auto& cfg = ConfigManager::getInstance();
    if (!cfg.load()) {
        LOG_ERROR("Failed to load config.yaml");
        return -1;
    }

    // 1. 初始化相机
    int cam_id = cfg.get<int>("camera.device_id", 0);
    int width = cfg.get<int>("camera.width", 640);
    int height = cfg.get<int>("camera.height", 480);
    Camera camera(cam_id, width, height);
    if (!camera.open()) {
        LOG_ERROR("Failed to open camera!");
        return -1;
    }
    float fx = cfg.get<float>("camera.focal_length_px", 600.0);
    camera.setFocalLength(fx);

    // 2. 初始化YOLO（若加载失败，detector将返回空检测）
    std::string model_path = cfg.get<std::string>("detector.model_path", "models/target.onnx");
    float conf_thresh = cfg.get<float>("detector.conf_threshold", 0.45);
    float nms_thresh = cfg.get<float>("detector.nms_threshold", 0.4);
    YOLODetector detector(model_path, conf_thresh, nms_thresh);

    // 3. 初始化跟踪器
    TargetTracker tracker;

    // 4. 初始化PID（修复：使用 cfg.getNode("pid")["yaw"] 方式）
    auto pid_node = cfg.getNode("pid");
    if (!pid_node.IsDefined()) {
        LOG_ERROR("PID config missing, using defaults");
    }
    auto yaw_node = pid_node["yaw"];
    auto pitch_node = pid_node["pitch"];
    // 若节点未定义，给默认值
    PIDParams yaw_params{1.8, 0.1, 0.3, 5.0, 25.0};
    PIDParams pitch_params{1.5, 0.08, 0.25, 5.0, 20.0};
    if (yaw_node.IsDefined()) {
        yaw_params.kp = yaw_node["kp"].as<double>(1.8);
        yaw_params.ki = yaw_node["ki"].as<double>(0.1);
        yaw_params.kd = yaw_node["kd"].as<double>(0.3);
        yaw_params.integral_limit = yaw_node["integral_limit"].as<double>(5.0);
        yaw_params.output_limit = yaw_node["output_limit"].as<double>(25.0);
    }
    if (pitch_node.IsDefined()) {
        pitch_params.kp = pitch_node["kp"].as<double>(1.5);
        pitch_params.ki = pitch_node["ki"].as<double>(0.08);
        pitch_params.kd = pitch_node["kd"].as<double>(0.25);
        pitch_params.integral_limit = pitch_node["integral_limit"].as<double>(5.0);
        pitch_params.output_limit = pitch_node["output_limit"].as<double>(20.0);
    }
    PIDController pid_yaw(yaw_params);
    PIDController pid_pitch(pitch_params);

    // 5. 初始化串口（容错：若打不开，只警告，继续运行）
    std::string port = cfg.get<std::string>("serial.port", "/dev/ttyUSB0");
    int baud = cfg.get<int>("serial.baudrate", 115200);
    SerialComm serial(port, baud);
    if (!serial.open()) {
        LOG_WARN("Serial port open failed, running in simulation mode.");
    }

    // 6. 主循环
    cv::Mat frame;
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    LOG_INFO("Vision Tracker Started!");
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time).count();
        last_time = current_time;
        if (dt > 0.1) dt = 0.033;

        if (!camera.read(frame)) {
            LOG_WARN("Frame read failed!");
            continue;
        }
        if (frame.empty()) continue;

        // 检测（若模型未加载，detect将返回空）
        auto detections = detector.detect(frame);
        //std::vector<Detection> detections;  // 空检测


        // ===== 临时模拟目标，训练好模型后删除 =====
        //static float sim_angle = 0.0f;
        //sim_angle += 0.015f;  // 移动速度
        //if (detections.empty()) {
        //    Detection sim_target;
        //    // 让目标在画面中央半径 150 像素的圆上运动
        //    sim_target.center.x = 320 + 150 * cos(sim_angle);
        //    sim_target.center.y = 240 + 100 * sin(sim_angle);
        //    sim_target.confidence = 0.9;
        //    detections.push_back(sim_target);
        //    LOG_INFO("SIMULATION MODE: Tracking virtual target");
        //}
        // ================================================================



        Detection target;
        bool has_target = false;
        if (!detections.empty()) {
            target = detections[0];
            has_target = true;
        }

        // 更新跟踪器
        if (has_target) {
            tracker.update(target, dt);
        } else {
            tracker.updateLost(dt);
        }

        cv::Point2f target_pos = tracker.getTargetPosition();
        TrackState state = tracker.getState();

        // 计算控制指令
        ControlCommand cmd{0, 0, false};
        if (state == TrackState::TRACKING && target_pos.x > 0 && target_pos.y > 0) {
            float cx = frame.cols / 2.0f;
            float cy = frame.rows / 2.0f;
            float dx = target_pos.x - cx;
            float dy = target_pos.y - cy;

            float angle_yaw = atan2(dx, fx);
            float angle_pitch = atan2(dy, fx);


            // ===== 临时放大角度 =====
            //float scale = 25.0f;
            //angle_yaw *= scale;
            //angle_pitch *= scale;
            // =========================



            float yaw_ctrl = pid_yaw.update(angle_yaw, dt);
            float pitch_ctrl = pid_pitch.update(angle_pitch, dt);

            cmd.yaw_angle = yaw_ctrl;
            cmd.pitch_angle = pitch_ctrl;
            //cmd.laser_enable = true;

            LOG_INFO("Frame {}: Target({:.1f},{:.1f}) | Yaw:{:.2f} Pitch:{:.2f}",
                     frame_count++, target_pos.x, target_pos.y, yaw_ctrl, pitch_ctrl);
        } else if (state == TrackState::SEARCHING) {
            cmd.yaw_angle = 2.0 * sin(current_time.time_since_epoch().count() * 1e-9);
            cmd.pitch_angle = 1.0 * cos(current_time.time_since_epoch().count() * 1e-9);
            //cmd.laser_enable = false;
        } else {
            cmd.yaw_angle = 0;
            cmd.pitch_angle = 0;
            //cmd.laser_enable = false;
        }

        if (serial.isOpen()) {

            // ====== 发送固定30度（测试后删除） ======
            //cmd.yaw_angle = 30.0;
            //cmd.pitch_angle = 0.0;
            // ====================================================

            serial.sendCommand(cmd);
        }

        // 可视化
        cv::Mat display = frame.clone();
        if (state == TrackState::TRACKING && target_pos.x > 0) {
            cv::circle(display, cv::Point(target_pos.x, target_pos.y), 5, cv::Scalar(0, 0, 255), -1);
        }
        cv::putText(display, "State: " + std::to_string(static_cast<int>(state)),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0));
        cv::imshow("Tracker", display);
        char key = cv::waitKey(1);
        if (key == 's') {
            static int save_count = 0;
            std::string filename = "dataset/images/raw/capture_" + std::to_string(save_count++) + ".jpg";
            cv::imwrite(filename, frame);
            LOG_INFO("Saved: {}", filename);
        }
        if (key == 'q') break;
    }

    serial.close();
    return 0;
}