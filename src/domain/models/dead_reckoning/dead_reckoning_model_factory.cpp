#include "dead_reckoning_model_factory.h"
#include <stdexcept>
#include <iostream>

std::shared_ptr<ideadReckoningModel> DeadReckoningModelFactory::create(
    const std::string& model_type,
    const AmrConfig& config
) 
{
    if (model_type == "differential_drive" || model_type == "dd") 
    {
        // config.amr_params에서 필요한 값 가져오기
        double wheel_base = config.amr_params.wheel_base;
        double wheel_radius = config.amr_params.wheel_radius;

        // (간단한 유효성 검사 예)
        if (wheel_base <= 0.0 || wheel_radius <= 0.0) {
            throw std::invalid_argument("Invalid wheel_base or wheel_radius in config");
        }

        return std::make_shared<deadReckoningEuler>(wheel_base, wheel_radius);
    }
    // else if (model_type == "ackermann") 
    // {
    // }
    else 
    {
        std::cerr << "[DeadReckoningModelFactory] Unknown model_type: '" << model_type << "'" << std::endl;
        throw std::invalid_argument("Unknown DeadReckoningModel type: " + model_type);
    }
}