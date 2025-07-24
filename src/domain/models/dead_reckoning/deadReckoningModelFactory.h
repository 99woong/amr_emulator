#pragma once

#include <memory>
#include <string>
#include "../infrastructure/yamlConfig.h"  // 전체 설정 구조체 선언 위치

#include "idead_reckoning.h"
#include "dead_reckoning_euler.h"
// #include "ackermannDeadReckoning.h" 등 필요시 추가

class DeadReckoningModelFactory 
{
public:
    static std::shared_ptr<ideadReckoningModel> create(
        const std::string& model_type,
        const AmrConfig& config  // 전체 config 전달
    );
};