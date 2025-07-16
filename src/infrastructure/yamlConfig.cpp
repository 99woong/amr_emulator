#include "YamlConfig.h"
#include <yaml-cpp/yaml.h>

AmrConfig YamlConfig::load(const std::string& filename) 
{
    YAML::Node config = YAML::LoadFile(filename);
    AmrConfig cfg;
    cfg.amr_count = config["amr_count"].as<int>();
    cfg.base_port = config["base_port"].as<int>();
    cfg.amr_params.wheel_radius = config["amr_params"]["wheel_radius"].as<double>();
    cfg.amr_params.wheel_base = config["amr_params"]["wheel_base"].as<double>();
    cfg.amr_params.max_speed = config["amr_params"]["max_speed"].as<double>();
    cfg.amr_params.max_angular_speed = config["amr_params"]["max_angular_speed"].as<double>();
    cfg.amr_params.max_acceleration = config["amr_params"]["max_acceleration"].as<double>();
    cfg.amr_params.max_deceleration = config["amr_params"]["max_deceleration"].as<double>();
    cfg.amr_params.max_angular_acceleration = config["amr_params"]["max_angular_acceleration"].as<double>();

    return cfg;
}