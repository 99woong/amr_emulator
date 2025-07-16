#include "imotorController.h"
#include "motorController.h"
#include "ddAccelerationModel.h"
#include <iostream>
#include <memory>
#include <iomanip>
#include "../src/infrastructure/yamlConfig.h"

#define STEP_COUNT  50

int main() 
{
    std::cout << "--- MotorController Test with DDAccelerationModel ---" << std::endl;

    //AmrConfig 파라미터설정(실제로는 YAML 파일에서 로드)
    AmrConfig config;
    config.amr_params.wheel_base = 0.5; 
    config.amr_params.max_speed = 2.0;  
    config.amr_params.max_angular_speed = 3.0; 
    config.amr_params.wheel_radius = 0.1; 

    // 가감속 모델 파라미터 (AmrConfig에 추가)
    config.dd_acceleration_params.mass_vehicle = 100.0; 
    config.dd_acceleration_params.load_weight = 20.0;  
    config.dd_acceleration_params.max_torque = 20.0;    
    config.dd_acceleration_params.friction_coeff = 0.05; 

    // 최종 제한할 가감속도 (모델 파라미터)
    config.dd_acceleration_params.max_acceleration = 1.5; 
    config.dd_acceleration_params.max_deceleration = 2.0; 
    config.dd_acceleration_params.max_angular_acceleration = 1.0; 
    config.dd_acceleration_params.max_angular_deceleration = 1.5; 


    // DDAccelerationModel 인스턴스 생성
    std::shared_ptr<AccelerationModel> dd_model = std::make_shared<DDAccelerationModel>(
        config.dd_acceleration_params.mass_vehicle,
        config.dd_acceleration_params.load_weight,
        config.dd_acceleration_params.max_torque,
        config.dd_acceleration_params.friction_coeff,
        config.amr_params.max_speed, 
        config.dd_acceleration_params.max_acceleration,
        config.dd_acceleration_params.max_deceleration,
        config.amr_params.wheel_radius, 
        config.dd_acceleration_params.max_angular_acceleration,
        config.dd_acceleration_params.max_angular_deceleration
    );  

    std::cout << "DDAccelerationModel created with parameters." << std::endl;

    // MotorController 인스턴스 생성
    MotorController motor_controller(config);
    std::cout << "MotorController created with config." << std::endl;

    // MotorController에 가감속 모델 설정 (주입)
    motor_controller.setAccelerationModel(dd_model);
    std::cout << "AccelerationModel set to MotorController." << std::endl;

    // 시뮬레이션 루프
    double dt = 0.05; 
    int total_steps = 100; 

    std::cout << "\n--- Scenario 1: Accelerating forward to 1.5 m/s, 0.0 rad/s ---" << std::endl;
    motor_controller.setVelocity(1.5, 0.0); 

    for (int i = 0; i < total_steps; ++i) {
        motor_controller.update(dt); 
    }

    std::cout << "\n--- Test Completed ---" << std::endl;

    return 0;
}