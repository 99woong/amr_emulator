#pragma once
#include "amr.h"
#include "tcp_server.h"
#include "yaml_config.h"
#include "iprotocol.h"
#include "vda5050_protocol.h"
#include "custom_tcp_protocol.h"
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
    
    // 새롭게 추가된 public 메서드: 등록된 프로토콜의 개수를 반환합니다.
    size_t getProtocolCount() const;
    // 특정 인덱스의 프로토콜을 가져오는 메서드 (필요한 경우 사용)
    IProtocol* getProtocol(size_t index) const;    
private:
    std::vector<std::unique_ptr<Amr>> amrs_;
    std::vector<std::unique_ptr<TcpServer>> servers_;
    std::vector<std::unique_ptr<IProtocol>> protocols_;
    bool isVda5050OrderMessage(const std::string& msg);
    bool isCustomTcpProtocolMessage(const std::string& msg);    
};