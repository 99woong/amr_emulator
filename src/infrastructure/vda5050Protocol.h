// #pragma once
// #include "../domain/interface/iprotocol.h"
// #include <vda5050++/interface_mc/connector.h>
// #include <mutex>

// class VDA5050ProtocolImpl : public IProtocol 
// {
//     vda5050pp::Connector connector_;
//     std::mutex mtx_;
//     std::string latestOrderId_;
// public:
//     VDA5050ProtocolImpl(const std::string& broker_url, const std::string& agv_id);
//     ~VDA5050ProtocolImpl();

//     std::string handleInbound(const std::string& msg) override;
//     void sendState(const std::string& stateMsg) override;
// };