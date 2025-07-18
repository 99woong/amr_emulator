#include "vda5050Protocol.h"
#include <iostream>

VDA5050ProtocolImpl::VDA5050ProtocolImpl(const std::string& broker_url, const std::string& agv_id)
    : connector_(
        vda5050pp::ConnectorConfig()
            .withMqttConfig({broker_url, 1883})
            .withAgvId(agv_id)
      )
{
    connector_.onOrder([this](std::shared_ptr<const vda5050::Order> order) {
        std::lock_guard<std::mutex> lock(mtx_);
        latestOrderId_ = order->orderId;
        std::cout << "[VDA5050] Received orderId=" << order->orderId << std::endl;
    });
    connector_.start();
}

VDA5050ProtocolImpl::~VDA5050ProtocolImpl() {
    connector_.stop();
}

std::string VDA5050ProtocolImpl::handleInbound(const std::string& msg) {
    if (msg == "get state") {
        std::lock_guard<std::mutex> lock(mtx_);
        return "order:" + latestOrderId_;
    }
    // 기타 커맨드 추가 구현 가능
    return "unsupported";
}

void VDA5050ProtocolImpl::sendState(const std::string& stateMsg) {
    vda5050::State state;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state.stateId = std::to_string(rand() % 1000);
        state.orderId = latestOrderId_;
    }
    state.operatingMode = vda5050::OperatingMode::AUTOMATIC;
    state.nodeId = "node_001";
    state.batteryState.batteryCharge = 0.9;
    connector_.publishState(state);
    std::cout << "[VDA5050] Published state: stateId=" << state.stateId << "\n";
}