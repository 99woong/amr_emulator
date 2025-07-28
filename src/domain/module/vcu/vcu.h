#pragma once
#include "ivcu.h"
#include "imotor_controller.h"
#include "inavigation.h"
#include "ilocalizer.h"  // Localizer 인터페이스 포함
#include <memory>
#include <vector>

class Vcu : public IVcu 
{
public:
    Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav, std::unique_ptr<ILocalizer> localizer);
    
    void setTargetPosition(double x, double y, double theta) override;
    void update(double dt) override;
    void updateNodes(const std::vector<NodeInfo>& nodes) override;
    void updateEdges(const std::vector<EdgeInfo>& edges) override;
    void setInitialPose(double x, double y, double theta) override;
    void getEstimatedPose(double& x, double& y, double& theta) const override;
    
    IMotorController& getMotor();
    INavigation& getNavigation();
    ILocalizer& getLocalizer();


private:
    std::unique_ptr<IMotorController> motor_;
    std::unique_ptr<INavigation> navigation_;
    std::unique_ptr<ILocalizer> localizer_;
    double target_x_, target_y_;
    double target_theta_;
};