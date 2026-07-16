#pragma once
#include <yaml-cpp/yaml.h>
#include <string>

class ConfigManager {
public:
    static ConfigManager& getInstance();
    bool load(const std::string& path = "config/config.yaml");
    
    template<typename T>
    T get(const std::string& key, T default_val = T()) const {
        try {
            return config_[key].as<T>();
        } catch(...) {
            return default_val;
        }
    }
    
    YAML::Node getNode(const std::string& key) const { return config_[key]; }

private:
    ConfigManager() = default;
    YAML::Node config_;
};