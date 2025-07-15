#pragma once
class IAmr 
{
public:
    virtual ~IAmr() = default;
    virtual void update() = 0;
    virtual int getId() const = 0;
};