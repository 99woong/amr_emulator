#pragma once
#include "IAmr.h"
#include "IVcu.h"
#include <memory>
class Amr : public IAmr 
{
public:
    Amr(int id, std::unique_ptr<IVcu> vcu);
    void update() override;
    int getId() const override;
    IVcu& getVcu();
private:
    int id_;
    std::unique_ptr<IVcu> vcu_;
};