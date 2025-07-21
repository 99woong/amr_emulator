// #pragma once
// #include "iprotocol.h"
// #include "iamr.h"

// // libvda5050pp include
// #include <events/order_event.h>
// #include <observer/order_observer.h>
// #include <memory>
// // #include <vector>

// class IAmr;
// class Vda5050Protocol : public IProtocol {
// public:
//     Vda5050Protocol();
//     void setAmr(IAmr* amr);

//     void start(); // 콜백 등록
//     void handleMessage(const std::string& msg, IAmr* amr) override;
//     std::string makeStateMessage(IAmr* amr) override;
// private:
//     IAmr* amr_;
//     vda5050pp::observer::OrderObserver order_observer_;
// };

#pragma once
#include "iprotocol.h"
#include "iamr.h"
#include <vda5050++/handle.h>
#include <vda5050++/config.h>
#include <memory>
#include <string>

class Vda5050Protocol : public IProtocol {
public:
    Vda5050Protocol();
    void setAmr(IAmr* amr);
    void setAgvId(const std::string& agv_id); // 각 인스턴스별 개별 agv_id
    void useDefaultConfig();                  // config 세팅
    void start() override;

    void handleMessage(const std::string& msg, IAmr* amr) override;
    std::string makeStateMessage(IAmr* amr) override;
private:
    IAmr* amr_ = nullptr;
    std::string agv_id_;
    vda5050pp::Config vda_config_;
    std::unique_ptr<vda5050pp::Handle> handle_;  // 핵심 라이브러리 객체
};