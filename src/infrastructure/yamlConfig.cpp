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

    // cfg.dd_acceleration_params.friction_coeff = config["acceleration_model"]["differential_drive"]["friction_coeff"].as<double>();
    // cfg.dd_acceleration_params.mass_vehicle = config["acceleration_model"]["differential_drive"]["mass_vehicle"].as<double>();
    // cfg.dd_acceleration_params.load_weight = config["acceleration_model"]["differential_drive"]["load_weight"].as<double>();
    // cfg.dd_acceleration_params.max_torque = config["acceleration_model"]["differential_drive"]["max_torque"].as<double>();
    // cfg.dd_acceleration_params.max_speed = config["acceleration_model"]["differential_drive"]["max_speed"].as<double>();
    // cfg.dd_acceleration_params.max_acceleration = config["acceleration_model"]["differential_drive"]["max_acceleration"].as<double>();
    // cfg.dd_acceleration_params.max_deceleration = config["acceleration_model"]["differential_drive"]["max_deceleration"].as<double>();

    return cfg;
}