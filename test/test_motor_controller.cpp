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

    // 1. AmrConfig 파라미터 설정 (실제로는 YAML 파일에서 로드)
    AmrConfig config;
    config.amr_params.wheel_base = 0.5; // 휠 간 거리 (m)
    config.amr_params.max_speed = 2.0;  // 로봇의 최대 선형 속도 (m/s)
    config.amr_params.max_angular_speed = 3.0; // 로봇의 최대 각속도 (rad/s)
    config.amr_params.wheel_radius = 0.1; // 휠 반경 (m)

    // 가감속 모델 파라미터 (AmrConfig에 추가)
    config.dd_acceleration_params.mass_vehicle = 100.0; // 차량 질량 (kg)
    config.dd_acceleration_params.load_weight = 20.0;   // 적재 중량 (kg)
    config.dd_acceleration_params.max_torque = 20.0;    // 모터 최대 토크 (Nm)
    config.dd_acceleration_params.friction_coeff = 0.05; // 마찰 계수

    // 물리 계산 후 최종 제한할 가감속도 (모델 파라미터)
    config.dd_acceleration_params.max_acceleration = 1.5; // m/s^2
    config.dd_acceleration_params.max_deceleration = 2.0; // m/s^2
    config.dd_acceleration_params.max_angular_acceleration = 1.0; // rad/s^2
    config.dd_acceleration_params.max_angular_deceleration = 1.5; // rad/s^2


    // 2. DDAccelerationModel 인스턴스 생성 (shared_ptr 사용)
    std::shared_ptr<AccelerationModel> dd_model = std::make_shared<DDAccelerationModel>(
        config.dd_acceleration_params.mass_vehicle,
        config.dd_acceleration_params.load_weight,
        config.dd_acceleration_params.max_torque,
        config.dd_acceleration_params.friction_coeff,
        config.amr_params.max_speed, // DDAccelerationModel도 자체 최대 속도 제한을 가짐
        config.dd_acceleration_params.max_acceleration,
        config.dd_acceleration_params.max_deceleration,
        config.amr_params.wheel_radius, // 휠 반경 전달
        config.dd_acceleration_params.max_angular_acceleration,
        config.dd_acceleration_params.max_angular_deceleration
    );  

    std::cout << "DDAccelerationModel created with parameters." << std::endl;

    // 3. MotorController 인스턴스 생성
    // MotorController는 AmrConfig 객체를 통해 초기화됩니다.
    MotorController motor_controller(config);
    std::cout << "MotorController created with config." << std::endl;

    // 4. MotorController에 가감속 모델 설정 (주입)
    motor_controller.setAccelerationModel(dd_model);
    std::cout << "AccelerationModel set to MotorController." << std::endl;

    // 5. 시뮬레이션 루프
    double dt = 0.05; // 시뮬레이션 시간 간격 (0.05초)
    int total_steps = 100; // 총 시뮬레이션 스텝

    std::cout << "\n--- Scenario 1: Accelerating forward to 1.5 m/s, 0.0 rad/s ---" << std::endl;
    motor_controller.setVelocity(1.5, 0.0); // 목표 속도 설정

    for (int i = 0; i < total_steps; ++i) {
        motor_controller.update(dt); // MotorController 업데이트
    }

    // std::cout << "\n--- Scenario 2: Decelerating to 0.0 m/s, 0.0 rad/s ---" << std::endl;
    // motor_controller.setVelocity(0.0, 0.0); // 정지 명령

    // for (int i = 0; i < total_steps; ++i) {
    //     motor_controller.update(dt);
    // }
    
    // std::cout << "\n--- Scenario 3: Accelerating to 1.0 m/s, 1.0 rad/s (simultaneously) ---" << std::endl;
    // motor_controller.setVelocity(1.0, 1.0); // 전진 및 회전 명령

    // for (int i = 0; i < total_steps; ++i) {
    //     motor_controller.update(dt);
    // }

    // std::cout << "\n--- Scenario 4: Decelerating to -0.5 m/s, -1.0 rad/s (reverse and turn) ---" << std::endl;
    // // 참고: 현재 DDAccelerationModel::applyAcceleration은 음수 속도를 허용하지 않으므로,
    // // 이 시나리오에서는 0.0 m/s로 감속만 일어날 것입니다.
    // // 후진을 허용하려면 applyAcceleration 함수를 수정해야 합니다.
    // motor_controller.setVelocity(-0.5, -1.0); 

    // for (int i = 0; i < total_steps; ++i) {
    //     motor_controller.update(dt);
    // }

    // std::cout << "\n--- Scenario 5: Reaching max_speed ---" << std::endl;
    // motor_controller.setVelocity(config.amr_params.max_speed, 0.0); // 최대 속도 도달 시도
    // for (int i = 0; i < total_steps; ++i) {
    //     motor_controller.update(dt);
    // }


    std::cout << "\n--- Test Completed ---" << std::endl;

    return 0;
}