#include "motorController.h"
#include <algorithm>
#include <cmath>
constexpr double PI = 3.14159265358979323846;

using namespace std;

MotorController::MotorController(const AmrConfig& config)
    : linear_vel_cmd_(0), angular_vel_cmd_(0),
      linear_vel_actual_(0), angular_vel_actual_(0),
      left_rpm_(0.0), right_rpm_(0.0),
      wheel_base_(config.amr_params.wheel_base), max_speed_(config.amr_params.max_speed), max_angular_speed_(config.amr_params.max_angular_speed),
      wheel_radius_(config.amr_params.wheel_radius)
    //   max_accel_(config.amr_params.max_acceleration), max_angular_accel_(config.amr_params.max_angular_acceleration)
{
    cout << "wheel_base_ : " << wheel_base_ << endl;
    cout << "max_speed_ : " << max_speed_ << endl;
    cout << "max_angular_speed_ : " << max_angular_speed_ << endl;
    cout << "wheel_radius_ : " << wheel_radius_ << endl;
}

void MotorController::setAccelerationModel(std::shared_ptr<AccelerationModel> model) {
    acceleration_model_ = model;
}

void MotorController::setVelocity(double linear, double angular) 
{
    cout << "linear : " << linear << "angular :" <<angular << endl; 
    
    // linear_vel_cmd_ = std::clamp(linear, -max_speed_, max_speed_);
    // angular_vel_cmd_ = std::clamp(angular, -max_angular_speed_, max_angular_speed_);
    linear_vel_cmd_ = linear;
    angular_vel_cmd_ = angular;

    // 최대 속도 제한
    linear_vel_cmd_ = std::clamp(linear_vel_cmd_, -max_speed_, max_speed_);
    angular_vel_cmd_ = std::clamp(angular_vel_cmd_, -max_angular_speed_, max_angular_speed_);
}

void MotorController::update(double dt) 
{
    if (acceleration_model_) 
    {
        // ✨ 가감속 모델을 사용하여 실제 속도 업데이트
        linear_vel_actual_ = acceleration_model_->applyAcceleration(linear_vel_actual_, linear_vel_cmd_, dt);
        angular_vel_actual_ = acceleration_model_->applyAngularAcceleration(angular_vel_actual_, angular_vel_cmd_, dt);
    } 
    else 
    {
        // 가감속 모델이 설정되지 않았다면, 목표 속도로 즉시 변경 (디버그 또는 폴백)
        linear_vel_actual_ = linear_vel_cmd_;
        angular_vel_actual_ = angular_vel_cmd_;
        std::cerr << "Warning: No AccelerationModel set for MotorController. Speeds updated instantly." << std::endl;
    }

    // 실제 휠 속도 계산 (차동 구동 로봇 기준)
    double left_wheel_speed, right_wheel_speed;
    calculateWheelSpeeds(linear_vel_actual_, angular_vel_actual_, left_wheel_speed, right_wheel_speed);

    // 휠 속도를 RPM으로 변환
    convertWheelSpeedToRPM(left_wheel_speed, left_rpm_);
    convertWheelSpeedToRPM(right_wheel_speed, right_rpm_);

    // 디버그 출력
    std::cout << "MotorController Update: "
              << "Cmd Lin: " << linear_vel_cmd_ << "m/s, "
              << "Actual Lin: " << linear_vel_actual_ << "m/s, "
              << "Cmd Ang: " << angular_vel_cmd_ << "rad/s, "
              << "Actual Ang: " << angular_vel_actual_ << "rad/s, "
              << "Left RPM: " << left_rpm_ << ", "
              << "Right RPM: " << right_rpm_ << std::endl;

    // double dt = 0.1; // [s], 주기
    // double linear_delta = linear_vel_cmd_ - linear_vel_actual_;
    // double linear_step = std::clamp(linear_delta, -max_accel_ * dt, max_accel_ * dt);
    // linear_vel_actual_ += linear_step;

    // double angular_delta = angular_vel_cmd_ - angular_vel_actual_;
    // double angular_step = std::clamp(angular_delta, -max_angular_accel_ * dt, max_angular_accel_ * dt);
    // angular_vel_actual_ += angular_step;

    // // v_left  = v - (w * L / 2)
    // // v_right = v + (w * L / 2)
    // double v_l = linear_vel_actual_ - (angular_vel_actual_ * wheel_base_ / 2.0);
    // double v_r = linear_vel_actual_ + (angular_vel_actual_ * wheel_base_ / 2.0);

    // cout << "v_l : " << v_l << "v_r : " << v_r << endl; 
    // // rpm = (v / (2 * PI * r)) * 60
    // left_rpm_ = (v_l / (2 * PI * wheel_radius_)) * 60.0;
    // right_rpm_ = (v_r / (2 * PI * wheel_radius_)) * 60.0;    


}

void MotorController::getRPM(double& left_rpm, double& right_rpm) const 
{
    left_rpm = left_rpm_;
    right_rpm = right_rpm_;
}

void MotorController::calculateWheelSpeeds(double linear_vel, double angular_vel, double& left_speed, double& right_speed) const {
    // 차동 구동 로봇의 휠 속도 계산 공식
    // linear_vel: 로봇의 전진 속도
    // angular_vel: 로봇의 회전 속도 (yaw rate)
    // wheel_base_: 휠 간의 거리
    left_speed = linear_vel - (angular_vel * wheel_base_ / 2.0);
    right_speed = linear_vel + (angular_vel * wheel_base_ / 2.0);
}

void MotorController::convertWheelSpeedToRPM(double wheel_speed, double& rpm) const {
    // 휠 속도 (m/s)를 RPM으로 변환
    // (속도 / (2 * pi * wheel_radius)) * 60
    if (wheel_radius_ > 0) {
        rpm = (wheel_speed / (2.0 * M_PI * wheel_radius_)) * 60.0;
    } else {
        rpm = 0.0;
        std::cerr << "Error: Wheel radius is zero or negative when converting to RPM." << std::endl;
    }
}