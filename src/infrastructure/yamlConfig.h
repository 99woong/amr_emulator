#pragma once
#include <string>
struct AmrParams 
{
    double wheel_diameter;
    double wheel_base;
    double max_angular_speed;
    double wheel_radius;
    double max_angular_acceleration;
    double mass_vehicle;      // kg
    double load_weight;       // kg
    double max_torque;        // Nm
    double friction_coeff;    // (0~1)
    double max_speed;         // m/s
    double max_acceleration;  // m/s^2
    double max_deceleration;  // m/s^2    

};

struct ddAccelerationParams {
    double mass_vehicle;      // kg
    double load_weight;       // kg
    double wheel_radius;      // m
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
    AmrParams amr_params;
    ddAccelerationParams dd_acceleration_params;
};

class YamlConfig 
{
public:
    static AmrConfig load(const std::string& filename);
};