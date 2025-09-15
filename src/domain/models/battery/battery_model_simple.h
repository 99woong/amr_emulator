#include "battery_model.h"
class BatteryModelSimple : public IBatteryModel 
{
public:
    BatteryModelSimple(double capacity_percent = 100.0, double idle_discharge_per_sec = 0.01,
                       double max_charge_per_sec = 1.0, double charge_stop_threshold = 95.0);
    
    void update(double dt, double linear_vel, double angular_vel, bool is_charging) override;
    double getCapacityPercent() const override;

private:
    double capacity_percent_;
    double idle_discharge_per_sec_;
    double max_charge_per_sec_;
    double charge_stop_threshold_;

    double calculateDischarge(double linear_vel, double angular_vel, double dt);
};