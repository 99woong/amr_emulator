#pragma once
#include <string>

class IAmr;

class IProtocol 
{
public:
    virtual ~IProtocol() = default;
    // FMS에서 들어온 메시지를 해석,처리(응답 반환)
    virtual void handleMessage(const std::string& msg, IAmr* amr) = 0;
    // AMR 상태 등 outbound 메시지 송신용 (실제 연결 처리 또는 모킹)
    virtual std::string makeStateMessage(IAmr* amr) = 0;
    virtual void start() = 0; 
};