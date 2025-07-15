#pragma once

class ILocalizer {
public:
    virtual ~ILocalizer() = default;

    // initial pose setting (x, y, yaw)
    virtual void setInitialPose(double x, double y, double theta) = 0;
    // input a current rpm(left, right, dt)
    virtual void update(double left_rpm, double right_rpm, double dt) = 0;
    // get a current pose (x, y, yaw)
    virtual void getPose(double& x, double& y, double& theta) const = 0;
};