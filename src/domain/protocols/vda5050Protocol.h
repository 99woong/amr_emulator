#pragma once
#include "iprotocol.h"
#include "iamr.h"

// libvda5050pp include
#include <events/order_event.h>
#include <observer/order_observer.h>
#include <memory>
// #include <vector>

class IAmr;
class Vda5050Protocol : public IProtocol {
public:
    Vda5050Protocol();
    void setAmr(IAmr* amr);

    void start(); // 콜백 등록
    void handleMessage(const std::string& msg, IAmr* amr) override;
    std::string makeStateMessage(IAmr* amr) override;
private:
    IAmr* amr_;
    vda5050pp::observer::OrderObserver order_observer_;
};
