#include "yamlConfig.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

AmrConfig YamlConfig::load(const std::string& filename) 
{
    YAML::Node config = YAML::LoadFile(filename);
    AmrConfig cfg;
    cfg.amr_count = config["amr_count"].as<int>();
    cfg.base_port = config["base_port"].as<int>();

    // Load protocol_type
    if (config["protocol_type"]) 
    {
        cfg.protocol_type = config["protocol_type"].as<std::string>();
    } 
    else 
    {
        cfg.protocol_type = "vda5050";
        std::cerr << "Warning: 'protocol_type' not found in config. Defaulting to 'vda5050'." << std::endl;
    }    

    cfg.amr_params.wheel_radius = config["amr_params"]["wheel_radius"].as<double>();
    cfg.amr_params.wheel_base = config["amr_params"]["wheel_base"].as<double>();
    cfg.amr_params.max_speed = config["amr_params"]["max_speed"].as<double>();
    cfg.amr_params.max_angular_speed = config["amr_params"]["max_angular_speed"].as<double>();
    cfg.amr_params.max_acceleration = config["amr_params"]["max_acceleration"].as<double>();
    cfg.amr_params.max_deceleration = config["amr_params"]["max_deceleration"].as<double>();
    cfg.amr_params.max_angular_acceleration = config["amr_params"]["max_angular_acceleration"].as<double>();

    if (config["dd_acceleration_params"]) 
    {
        cfg.dd_acceleration_params.mass_vehicle = config["dd_acceleration_params"]["mass_vehicle"].as<double>();
        cfg.dd_acceleration_params.load_weight = config["dd_acceleration_params"]["load_weight"].as<double>();
        cfg.dd_acceleration_params.wheel_radius = config["dd_acceleration_params"]["wheel_radius"].as<double>();
        cfg.dd_acceleration_params.max_torque = config["dd_acceleration_params"]["max_torque"].as<double>();
        cfg.dd_acceleration_params.friction_coeff = config["dd_acceleration_params"]["friction_coeff"].as<double>();
        cfg.dd_acceleration_params.max_speed = config["dd_acceleration_params"]["max_speed"].as<double>();
        cfg.dd_acceleration_params.max_acceleration = config["dd_acceleration_params"]["max_acceleration"].as<double>();
        cfg.dd_acceleration_params.max_deceleration = config["dd_acceleration_params"]["max_deceleration"].as<double>();
        cfg.dd_acceleration_params.max_angular_acceleration = config["dd_acceleration_params"]["max_angular_acceleration"].as<double>();
        cfg.dd_acceleration_params.max_angular_deceleration = config["dd_acceleration_params"]["max_angular_deceleration"].as<double>();
    }    
    return cfg;
}