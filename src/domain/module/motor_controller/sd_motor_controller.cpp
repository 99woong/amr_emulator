#include "sd_motor_controller.h"
#include <algorithm>
#include <cmath>
constexpr double PI = 3.14159265358979323846;

using namespace std;

SDMotorController::SDMotorController(const AmrConfig& config)
    : linear_vel_cmd_(0), angular_vel_cmd_(0),
      linear_vel_actual_(0), angular_vel_actual_(0),
      left_rpm_(0.0), right_rpm_(0.0), steering_angle_cmd_(0.0), front_wheel_speed_cmd_(0.0), max_steering_angle_(30.0),
      wheel_base_(config.amr_params.wheel_base), max_speed_(config.amr_params.max_speed), max_angular_speed_(config.amr_params.angularSpeedMax),
      wheel_radius_(config.amr_params.wheel_radius)
{
    cout << "wheel_base_ : " << wheel_base_ << endl;
    cout << "max_speed_ : " << max_speed_ << endl;
    cout << "max_angular_speed_ : " << max_angular_speed_ << endl;
    cout << "wheel_radius_ : " << wheel_radius_ << endl;
}

void SDMotorController::setAccelerationModel(std::shared_ptr<AccelerationModel> model) {
    acceleration_model_ = model;
}

void SDMotorController::setMaxSpeed(double max_speed)
{
    max_speed_ = max_speed;
}

void SDMotorController::setVelocity(double linear, double angular) 
{
    // cout << "[SD_MotorController::setVelocity] linear : " << linear << " angular :" <<angular << endl; 
    if (std::abs(linear) > 1e-5) 
    {
        steering_angle_cmd_ = std::atan(wheel_base_ * angular / linear);
    } 
    else 
    {
        steering_angle_cmd_ = 0.0;
    }

    front_wheel_speed_cmd_ = linear; 
    // 제한 걸기
    steering_angle_cmd_ = std::clamp(steering_angle_cmd_, -max_steering_angle_, max_steering_angle_);
    front_wheel_speed_cmd_ = std::clamp(front_wheel_speed_cmd_, -max_speed_, max_speed_);    
    // cout << "[SD_MotorController::setVelocity] " << steering_angle_cmd_ << " " <<front_wheel_speed_cmd_ << endl; 
}

double SDMotorController::getLinearVelocity() const
{
    // std::cout << " SDMotorController::getLinearVelocity(): " << front_linear_vel_actual_<<std::endl;
    return front_linear_vel_actual_;
}
double SDMotorController::getAngularVelocity() const
{
    // std::cout << " SDMotorController::getAngularVelocity(): " << steering_angular_vel_actual_<<std::endl;
    return steering_angular_vel_actual_;
}
    

void SDMotorController::update(double dt) 
{
    if (acceleration_model_) 
    {
        // 가감속 모델을 사용하여 실제 속도 업데이트
        front_linear_vel_actual_ = acceleration_model_->applyAcceleration(front_linear_vel_actual_, front_wheel_speed_cmd_, dt);
        steering_angular_vel_actual_ = acceleration_model_->applyAngularAcceleration(steering_angular_vel_actual_, steering_angle_cmd_, dt);
    } 
    else 
    {
        // 가감속 모델이 설정되지 않았다면, 목표 속도로 즉시 변경 (디버그 또는 폴백)
        front_linear_vel_actual_ = front_wheel_speed_cmd_;
        steering_angular_vel_actual_ = steering_angle_cmd_;
        // std::cerr << "Warning: No AccelerationModel set for MotorController. Speeds updated instantly." << std::endl;
    }

    // 실제 휠 속도 계산 (차동 구동 로봇 기준)
    double left_wheel_speed, right_wheel_speed;

    left_wheel_speed = front_linear_vel_actual_;
    right_rpm_ = steering_angular_vel_actual_;
    
    // 휠 속도를 RPM으로 변환
    convertWheelSpeedToRPM(left_wheel_speed, left_rpm_);
    // convertWheelSpeedToRPM(right_wheel_speed, right_rpm_);
}

void SDMotorController::getRPM(double& front_wheel_rpm, double& front_steering_angle) const 
{
    front_wheel_rpm = left_rpm_;
    front_steering_angle = right_rpm_;
}

void SDMotorController::calculateWheelSpeeds(double linear_vel, double angular_vel, double& left_speed, double& right_speed) const 
{
    // return (front_wheel_speed_cmd_ / (2.0 * M_PI * wheel_radius_)) * 60.0;
}

void SDMotorController::convertWheelSpeedToRPM(double wheel_speed, double& rpm) const 
{
    if (wheel_radius_ > 0) 
    {
        rpm = (wheel_speed / (2.0 * M_PI * wheel_radius_)) * 60.0;
    } 
    else
    {
        rpm = 0.0;
        std::cerr << "Error: Wheel radius is zero or negative when converting to RPM." << std::endl;
    }

    // std::cout << "[convertWheelSpeedToRPM] : " << wheel_speed << " " << rpm << std::endl;

}

