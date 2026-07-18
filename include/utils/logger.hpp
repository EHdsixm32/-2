#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <memory>

class Logger {
public:
    static void init();
    static std::shared_ptr<spdlog::logger> get() { return logger_; }
private:
    static std::shared_ptr<spdlog::logger> logger_;
};

#define LOG_INFO(...)   Logger::get()->info(__VA_ARGS__)
#define LOG_WARN(...)   Logger::get()->warn(__VA_ARGS__)
#define LOG_ERROR(...)  Logger::get()->error(__VA_ARGS__)