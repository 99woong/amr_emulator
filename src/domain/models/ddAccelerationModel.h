#pragma once

#include "accelerationModel.h" // 추상 클래스 포함
#include <string>

/**
 * @brief 차동 구동(Differential Drive) 로봇을 위한 가감속 모델입니다.
 * 이 모델은 선형 및 각 가감속을 계산하기 위한 파라미터를 사용합니다.
 */
class DDAccelerationModel : public AccelerationModel {
public:
    /**
     * @brief DDAccelerationModel의 생성자입니다.
     * @param mass_vehicle 차량 질량 (kg).
     * @param load_weight 적재 중량 (kg).
     * @param max_torque 최대 토크 (Nm).
     * @param friction_coeff 마찰 계수.
     * @param max_speed 최대 선형 속도 (m/s).
     * @param max_acceleration 최대 선형 가속도 (m/s^2).
     * @param max_deceleration 최대 선형 감속도 (m/s^2).
     * @param wheel_radius 휠 반경 (m). (새롭게 추가된 파라미터)
     * @param max_angular_acceleration 최대 각가속도 (rad/s^2).
     * @param max_angular_deceleration 최대 각감속도 (rad/s^2).
     */
    DDAccelerationModel(
        double mass_vehicle,
        double load_weight,
        double max_torque,
        double friction_coeff,
        double max_speed,
        double max_acceleration,
        double max_deceleration,
        double wheel_radius, // 새로 추가된 파라미터
        double max_angular_acceleration,
        double max_angular_deceleration
    );

    /**
     * @brief 현재 속도에서 목표 속도까지의 가속을 DD 모델에 따라 적용합니다.
     * 물리 기반의 힘 계산을 포함합니다.
     * @param current_speed 현재 속도 (m/s).
     * @param target_speed 목표 속도 (m/s).
     * @param dt 시간 간격 (초).
     * @return dt 시간 간격 후의 새로운 속도 (m/s).
     */
    double applyAcceleration(double current_speed, double target_speed, double dt) override;

    /**
     * @brief 현재 각속도에서 목표 각속도까지의 각가속을 DD 모델에 따라 적용합니다.
     * @param current_angular_speed 현재 각속도 (rad/s).
     * @param target_angular_speed 목표 각속도 (rad/s).
     * @param dt 시간 간격 (초).
     * @return dt 시간 간격 후의 새로운 각속도 (rad/s).
     */
    double applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) override;

    /**
     * @brief YAML 파일에서 모델 파라미터를 로드합니다. (구현 예정)
     * @param yaml_file_path YAML 파일의 경로.
     * @return 로드 성공 여부.
     */
    bool loadParametersFromYaml(const std::string& yaml_file_path) override;

private:
    // 가감속 모델 파라미터
    double mass_vehicle_;       // 차량 질량
    double load_weight_;        // 적재 중량
    double max_torque_;         // 최대 토크 (Nm)
    double friction_coeff_;     // 마찰 계수
    double max_speed_;          // 최대 선형 속도
    double max_acceleration_;   // 최대 선형 가속도 (물리 계산 후 적용될 최대값)
    double max_deceleration_;   // 최대 선형 감속도 (물리 계산 후 적용될 최대값)
    double wheel_radius_;       // ✨ 휠 반경 (m)
    double max_angular_acceleration_; // 최대 각가속도
    double max_angular_deceleration_; // 최대 각감속도

    const double GRAVITY = 9.81; // 중력 가속도 (m/s^2)
};
