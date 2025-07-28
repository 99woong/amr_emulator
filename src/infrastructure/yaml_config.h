#pragma once
#include <string>
struct AmrParams 
{
    double wheel_diameter;
    double wheel_base;
    double max_angular_speed;
    double wheel_radius;
    double mass_vehicle;      // kg
    double load_weight;       // kg
    double max_torque;        // Nm
    double friction_coeff;    // (0~1)
    double max_speed;         // m/s
    double max_acceleration;  // m/s^2
    double max_deceleration;  // m/s^2    
    double max_angular_acceleration;  // m/s^2
    double max_angular_deceleration;  // m/s^2
};

struct AmrConfig 
{
    int amr_count;
    int base_port;
    std::string protocol_type;   
    std::string vehicle_type;  
    std::string dead_reckoning_model;  
    double speedup_ratio = 1.0;
    AmrParams amr_params;
};

class YamlConfig 
{
public:
    static AmrConfig load(const std::string& filename);
};