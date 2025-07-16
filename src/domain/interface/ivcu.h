#pragma once
class IVcu 
{
public:
    virtual ~IVcu() = default;
    virtual void setTargetPosition(double x, double y) = 0;
    virtual void update() = 0;
};
