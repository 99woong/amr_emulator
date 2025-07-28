# 소개
amr_emulator는 AMR을 가상환경에서 시뮬레이션할 때, AMR 동특성을 모방하여 정교한 시뮬레이션 가능하게 함

# 기대효과
###  개발속도 가속화
  - 물리적 AMR 없이, 조기 통합 테스트 활성화
  - 주요 모듈(통신,제어,센서) 손쉽게 교체함으로써 모듈간 상호 영향없이 AMR 빠른 검증 가능
###  정교한 시뮬레이션
  - AMR의 움직임, 다양한 센서 데이터(예: 라이다, 카메라, 초음파 등),배터리소모량과 같은 물리적 특성을 실제와 거의 흡사하게 모델링
  - 실제 환경에서 발생할 수 있는 복잡한 시나리오를 가상 환경에서 재현하고 테스트 가능
  - AMR이 배포되었을 때 발생할 수 있는 예측 불가능한 상황을 최소화, 시스템의 신뢰도 향상 가능


# Requirement / 기능명세
![Diagram](image/amr_emulator_diagram.png)
## 모듈화 아키텍처 기반으로 설계
- 각 기능은 독립적인 모듈로 구현되어 모듈 변경 시 다른 모듈에 미치는 영향을 최소화해야 함
- 유지보수성을 높이고, 특정 모듈의 기능 개선 또는 교체가 용이하도록 함
  - AMR (Autonomous Mobile Robot): 전반적인 로봇 제어 및 통합 담당
  - VCU (Vehicle Control Unit): 차량의 구동 및 제어 시스템 담당
  - Navigator: 경로 계획 및 추종 
  - Localizer: 현재 로봇 위치 추정 
  - Motor Controller: 로봇의 속도에 따른 모터 제어
  - Accelerator: 가속 및 감속 모델 담당
  - dead reckoning : 로봇의 이번 위치와 현재 센서값으로 현재 위치 예측

imotor_controller.h
```
class IMotorController 
{
public:
    virtual ~IMotorController() = default;
    virtual void setAccelerationModel(std::shared_ptr<AccelerationModel> model) = 0;
    virtual void setVelocity(double linear, double angular) = 0;
    virtual void update(double dt) = 0;
    virtual void getRPM(double& left_rpm, double& right_rpm) const = 0;
};
```
motor_controller.h
```
class MotorController : public IMotorController
{
public:
    MotorController(const AmrConfig& config);
    
    void setAccelerationModel(std::shared_ptr<AccelerationModel> model) override;
    void setVelocity(double linear, double angular) override;
    void update(double dt) override;
    void getRPM(double& left_rpm, double& right_rpm) const override;

private:
    std::shared_ptr<AccelerationModel> acceleration_model_;

    double linear_vel_cmd_, angular_vel_cmd_;
    double linear_vel_actual_, angular_vel_actual_;
    double left_rpm_, right_rpm_;
    double wheel_base_, max_speed_, max_angular_speed_, wheel_radius_;
    double max_accel_, max_angular_accel_;

    void calculateWheelSpeeds(double linear_vel, double angular_vel, double& left_speed, double& right_speed) const;
    void convertWheelSpeedToRPM(double wheel_speed, double& rpm) const;
};
```
## 차량 물리특성기반 가감속 모델 구현
-  다양한 AMR 유형에 대한 시뮬레이션 유연성 확보를 위해, 차량종류 및 물리적특성을 파라미터로 로딩하여 현실적인 가감속 모델을 구현
```
vehicle_type: "differential_driver"  # 차량 종류 (예: 차동 구동 방식)
mass_vehicle: 1500.0                # 차량 자체 질량 (kg)
load_weight: 500.0                  # 적재 중량 (kg)
max_torque: 0.3                     # 최대 토크 (Nm)
friction_coeff: 0.015               # 마찰 계수
max_speed: 2.0                      # 최대 속도 (m/s)
max_acceleration: 1.0               # 최대 가속도 (m/s^2)
max_deceleration: 1.5               # 최대 감속도 (m/s^2)
```

## 차량 종류별 추측 항법 및 노이즈 모델
- 차량 종류에 맞는 추측 항법(Dead Reckoning) 알고리즘을 적용하고, 실제 센서 오차를 근사화하기 위해 가우시안 노이즈를 추가
- 이를 통해 보다 현실적인 시뮬레이션 환경을 제공하고, 실제 로봇의 동작을 예측하고 검증 가능
```
vehicle_type: "differential_driver"     # 차량 종류
dead_reckoning_model: "rk4"             # 추측 항법 모델 (예: 룽게-쿠타 4차)
gaussian_noise_level:                   # 가우시안 노이즈 레벨 (구체적인 값 필요)
  position_std_dev: 0.01                # 위치 표준 편차 (m)
  orientation_std_dev: 0.005            # 방향 표준 편차 (rad)
```

```
│   │   └── dead_reckoning       # 추측항법 알고리즘(오일러,룽게-쿠타 등)을 사용하여 로봇의 이동 위치를 예측, 교체 가능
│   │       ├── dead_reckoning_euler.cpp
│   │       ├── dead_reckoning_euler.h
│   │       ├── deadReckoningModelFactory.cpp
│   │       ├── deadReckoningModelFactory.h
│   │       ├── dead_reckoning_rk2.CPP
│   │       ├── dead_reckoning_rk2.h
│   │       ├── dead_reckoning_rk4.cpp
│   │       ├── dead_reckoning_rk4.h
│   │       └── idead_reckoning.h
```
## 시뮬레이션 배속 기능
- 개발,테스트과정에서 시뮬레이션 시간을 단축하거나, 특정상황을 더 상세히 분석하기 위해 속도를 조절 필요
- 에뮬레이터는 시뮬레이션 배속 설정에 따라 내부 동작 속도 조정
- 시뮬레이션 배속 설정은 YAML 파일을 통해 외부에서 쉽게 지정할 수 있도록 구현
```
speedup_ratio: 1
:
const double base_dt = 1.0; // 기존 1초 루프
const double sim_dt = base_dt / config.speedup_ratio; // 배속에 따라 루프주기 단축
:
auto& amrs = manager.getAmrs();
for (size_t i = 0; i < amrs.size(); ++i)
{
   amrs[i]->step(sim_dt);
:
void Amr::step(double dt) 
{
    if (!nodes_.empty()) 
    {
        vcu_->update(dt);
:
void Vcu::update(double dt) 
{
    double left_rpm, right_rpm;
    motor_->getRPM(left_rpm, right_rpm);

    localizer_->update(left_rpm, right_rpm, dt);
    
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    localizer_->getPose(current_x, current_y, current_theta);

    double linear_vel_cmd = 0.0, angular_vel_cmd = 0.0;
    navigation_->update(current_x, current_y, current_theta, linear_vel_cmd, angular_vel_cmd);

    motor_->setVelocity(linear_vel_cmd, angular_vel_cmd);
    motor_->update(dt);
}           
```


```


amr_emulator
├── config                # 코드 변경 없이 애플리케이션의 동작을 유연하게 제어하기 위한 amr 설정파라메터(yaml, json, xml, ini,..)
├── image
├── src                   # amr 애플리케이션의 모든 소스 코드를 포함하는 핵심 디렉토리
│   ├── app                    # domain계층 로직들을 활용, 특정 use case 또는 애플리케이션의 흐름 정의
│   ├── domain                 # 프로젝트의 가장 핵심적인 부분, 에뮬레이터가 "무엇을 하는지"에 대한 순수한 로직과 데이터 모델 관리   
│   ├── infrastructure         # 애플리케이션을 구동하는 데 필요한 외부 시스템(DB, 네트워크, 설정 로딩) 담당
│   └── presentation           # 애플리케이션이 사용자 또는 외부 시스템에 보여주는 방식을 담당(GUI, 웹대시보드, REST API,..) 
├── test                  # amr_emulator 프로젝트내의 각 기능별 테스트 코드 관리
│   └── fms_test
└── third_party           # 프로젝트에서 사용되는 외부 라이브러리 및 종속성의 소스코드나 바이너리 파일 관리
    ├── libvda5050pp
    └── yaml-cpp
```

```
src
├── app
│   ├── amrManager.cpp           # 여러 AMR 인스턴스, 프로토콜 라우팅, 서버 인스턴스 관리, 전체 시스템의 시작/중지
│   └── amrManager.h
├── domain
│   ├── common
│   │   └── NodeEdgeInfo.h
│   ├── models    #AMR의 움직임,위치추정,제어 등 AMR자체의 핵심적인 동작방식과 관련된 수학적/물리적 모델 정의
│   │   ├── acceleration         # 로봇의 물리적 특성이나 환경변화에 따른 가속도 모델을 독립적으로 관리하고 교체 가능  
│   │   │   ├── accelerationModel.h
│   │   │   ├── ddAccelerationModel.cpp
│   │   │   └── ddAccelerationModel.h
│   │   └── dead_reckoning       # 추측항법 알고리즘(오일러,룽게-쿠타 등)을 사용하여 로봇의 이동 위치를 예측, 교체 가능
│   │       ├── dead_reckoning_euler.cpp
│   │       ├── dead_reckoning_euler.h
│   │       ├── deadReckoningModelFactory.cpp
│   │       ├── deadReckoningModelFactory.h
│   │       ├── dead_reckoning_rk2.CPP
│   │       ├── dead_reckoning_rk2.h
│   │       ├── dead_reckoning_rk4.cpp
│   │       ├── dead_reckoning_rk4.h
│   │       └── idead_reckoning.h
│   ├── module
│   │   ├── amr                  # 단일 AMR 인스턴스의 행동관리 및 상태제공,로봇의 이동 로직 실행
│   │   │   ├── amr.cpp
│   │   │   ├── amr.h
│   │   │   └── iamr.h
│   │   ├── localizer            # dead reckoning을 포함하여 더 넓은 의미의 위치결정 로직(필터기반, SLAM,..)
│   │   │   ├── ilocalizer.h
│   │   │   ├── localizer.cpp
│   │   │   └── localizer.h
│   │   ├── motorContorller      # AMR의 모터 구동 및 제어 로직            
│   │   │   ├── imotorController.h
│   │   │   ├── motorController.cpp
│   │   │   └── motorController.h
│   │   ├── navigator            # AMR이 목표 지점까지 효율적으로 이동할 수 있도록 경로 계획 및 이동 지시를 내리는 로직
│   │   │   ├── inavigation.h
│   │   │   ├── navigation.cpp
│   │   │   └── navigation.h
│   │   └── vcu                  # 차량제어 수행. motorController,navigator,localizer 등 하위 모듈기능을 통합 조율
│   │       ├── ivcu.h
│   │       ├── vcu.cpp
│   │       └── vcu.h
│   └── protocols        # AMR과 외부 시스템 간의 통신프로토콜 로직, VDA 5050 및 사용자정의TCP 프로토콜 선택 가능  
│       ├── customTcpProtocol.cpp
│       ├── customTcpProtocol.h
│       ├── iprotocol.h
│       ├── vda5050Protocol.cpp
│       └── vda5050Protocol.h
├── infrastructure    
│   ├── itcpServer.h
│   ├── tcpServer.cpp
│   ├── tcpServer.h
│   ├── yamlConfig.cpp
│   └── yamlConfig.h
├── main.cpp
└── presentation      
    ├── amrServer.cpp
    └── amrServer.h
```

# 설치
# 사용법
# 테스트
# Todo
 1. 개발환경 및 레포지토리 구축 - done
 2. 기능정의 설계 - done
 3. 프로젝트 폴더 및 파일구조 설계 - done
 4. Motor controller - done 
 5. FMS
 6. Localizer
 7. Navigation
 8. Obstacle detection
 9. Battery management
 10. 통합 테스트
 
# 문의
