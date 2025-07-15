#pragma once
#include "../domain/Amr.h"
#include "../infrastructure/TcpServer.h"
#include "../infrastructure/yamlConfig.h"
#include <vector>
#include <memory>
class AmrManager 
{
public:
    AmrManager(const AmrConfig& config);
    void startAll();
    void stopAll();
private:
    std::vector<std::unique_ptr<Amr>> amrs_;
    std::vector<std::unique_ptr<TcpServer>> servers_;
};