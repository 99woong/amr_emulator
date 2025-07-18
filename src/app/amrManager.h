#pragma once
#include "../domain/amr.h"
#include "../infrastructure/tcpServer.h"
#include "../infrastructure/yamlConfig.h"
#include "../domain/interface/iprotocol.h"
#include "../domain/protocols/vda5050Protocol.h"
#include <vector>
#include <memory>
class AmrManager 
{
public:
    AmrManager(const AmrConfig& config);
    void startAll();
    void stopAll();
    std::vector<std::unique_ptr<Amr>>& getAmrs() 
    { 
        return amrs_; 
    }
private:
    std::vector<std::unique_ptr<Amr>> amrs_;
    std::vector<std::unique_ptr<TcpServer>> servers_;
    std::vector<std::unique_ptr<IProtocol>> protocols_;
};