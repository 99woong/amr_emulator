#include "battery_model_simple.h"
#include <algorithm>
#include <cmath>

BatteryModelSimple::BatteryModelSimple(double capacity_percent, double idle_discharge_per_sec,
                           double max_charge_per_sec, double charge_stop_threshold)
    : capacity_percent_(capacity_percent),
      idle_discharge_per_sec_(idle_discharge_per_sec),
      max_charge_per_sec_(max_charge_per_sec),
      charge_stop_threshold_(charge_stop_threshold)
{
}

void BatteryModelSimple::update(double dt, double linear_vel, double angular_vel, bool is_charging)
{
    if (is_charging) 
    {
        if (capacity_percent_ < charge_stop_threshold_) 
        {
            capacity_percent_ += max_charge_per_sec_ * dt;
            capacity_percent_ = std::min(capacity_percent_, 100.0);
        }
    } 
    else 
    {
        if (std::abs(linear_vel) < 1e-3 && std::abs(angular_vel) < 1e-3) 
        {
            // idle 상태 방전
            capacity_percent_ -= idle_discharge_per_sec_ * dt;
        } 
        else 
        {
            // 주행 속도에 따른 방전
            capacity_percent_ -= calculateDischarge(linear_vel, angular_vel, dt);
        }

        if (capacity_percent_ < 0.0) capacity_percent_ = 0.0;
    }
}

double BatteryModelSimple::calculateDischarge(double linear_vel, double angular_vel, double dt)
{
    // 선속도 0~2m/s 구간: 0.05%/초, 2~4m/s 구간: 0.1%/초, 4m/s 이상: 0.2%/초
    double linear_factor = 0.0;
    double abs_linear = std::abs(linear_vel);
    if (abs_linear < 2.0) 
    {
        linear_factor = 0.05;
    } 
    else if(abs_linear < 4.0) 
    {
        linear_factor = 0.1;
    } 
    else 
    {
        linear_factor = 0.2;
    }

    // 각속도 방전 반영 (rad/s 당 0.02%/초 추가)
    double angular_factor = 0.02 * std::abs(angular_vel);

    double discharge_rate_per_sec = linear_factor + angular_factor;

    return discharge_rate_per_sec * dt;
}

double BatteryModelSimple::getCapacityPercent() const
{
    return capacity_percent_;
}