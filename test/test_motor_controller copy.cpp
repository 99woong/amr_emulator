#include "imotorController.h"
#include "motorController.h"
#include <iostream>
#include <memory>
#include <iomanip>
#include "../src/infrastructure/yamlConfig.h"

#define STEP_COUNT  50

int main() 
{
    std::string config_path = "C:/Users/ZENIX/Documents/src/amr_emulator/config/amr_params.yaml";
    AmrConfig config = YamlConfig::load(config_path);
    
    // params: wheel_base, max_speed, max_angular_speed, wheel_radius, max_accel, max_angular_accel
    auto motor = std::make_unique<MotorController>(config);

    motor->setVelocity(1.2, 1.2);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Step\tLeft RPM\tRight RPM\n";
    for (int i = 0; i < STEP_COUNT; ++i) 
    {
        motor->update(0.05);
        double l_rpm, r_rpm;
        motor->getRPM(l_rpm, r_rpm);
        std::cout << i << "\t" << l_rpm << "\t\t" << r_rpm << std::endl;
    }

    motor->setVelocity(0.0, 0.0);
    std::cout << "\nStop Command Issued\n";
    for (int i = 0; i < STEP_COUNT; ++i) 
    {
        motor->update(0.05);
        double l_rpm, r_rpm;
        motor->getRPM(l_rpm, r_rpm);
        std::cout << i << "\t" << l_rpm << "\t\t" << r_rpm << std::endl;
    }

    return 0;
}