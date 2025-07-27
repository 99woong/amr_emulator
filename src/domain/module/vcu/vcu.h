#pragma once
#include "ivcu.h"
#include "imotor_controller.h"
#include "inavigation.h"
#include <memory>
#include <vector>

class Vcu : public IVcu 
{
public:
    Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav);
    void setTargetPosition(double x, double y) override;
    void update() override;
    IMotorController& getMotor();
    INavigation& getNavigation();

    void updateNodes(const std::vector<NodeInfo>& nodes) override;
    void updateEdges(const std::vector<EdgeInfo>& edges) override;    
private:
    std::unique_ptr<IMotorController> motor_;
    std::unique_ptr<INavigation> navigation_;
    double target_x_, target_y_;
};