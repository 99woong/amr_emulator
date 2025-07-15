#ifndef ACCELERATION_MODEL_HPP
#define ACCELERATION_MODEL_HPP

#include <string>
#include <memory> // For std::shared_ptr

/**
 * @brief 가감속 모델의 추상 기본 클래스입니다.
 * 이 클래스는 로봇의 선형 및 각 가감속 동작을 정의하는 인터페이스를 제공합니다.
 */
class AccelerationModel {
public:
    /**
     * @brief 가상 소멸자입니다. 파생 클래스의 올바른 소멸을 보장합니다.
     */
    virtual ~AccelerationModel() = default;

    /**
     * @brief 현재 속도에서 목표 속도까지의 가속을 적용하여 새로운 속도를 계산합니다.
     * @param current_speed 현재 속도 (m/s).
     * @param target_speed 목표 속도 (m/s).
     * @param dt 시간 간격 (초).
     * @return dt 시간 간격 후의 새로운 속도 (m/s).
     */
    virtual double applyAcceleration(double current_speed, double target_speed, double dt) = 0;

    /**
     * @brief 현재 각속도에서 목표 각속도까지의 각가속을 적용하여 새로운 각속도를 계산합니다.
     * @param current_angular_speed 현재 각속도 (rad/s).
     * @param target_angular_speed 목표 각속도 (rad/s).
     * @param dt 시간 간격 (초).
     * @return dt 시간 간격 후의 새로운 각속도 (rad/s).
     */
    virtual double applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) = 0;

    /**
     * @brief 모델의 파라미터를 YAML 파일에서 로드하는 추상 메서드.
     * @param yaml_file_path YAML 파일의 경로.
     * @return 로드 성공 여부.
     */
    virtual bool loadParametersFromYaml(const std::string& yaml_file_path) = 0;
};

#endif // ACCELERATION_MODEL_HPP
