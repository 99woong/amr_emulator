#include "Amr.h"
Amr::Amr(int id, std::unique_ptr<IVcu> vcu) : id_(id), vcu_(std::move(vcu)) 
{
    
}
void Amr::update() 
{ 
    vcu_->update(); 
}
int Amr::getId() const 
{ 
    return id_; 
}
IVcu& Amr::getVcu() 
{ 
    return *vcu_; 
}