#pragma once

#include "realsense.hpp"

#include <memory>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/discovery.hpp>

#include <librealsense2/rs.hpp>

namespace realsense {
namespace discovery {

template <typename ContextT>
class RealsenseDiscovery : public viam::sdk::Discovery {
  std::shared_ptr<ContextT> rs_ctx_;

public:
  static inline viam::sdk::Model model{"viam", "realsense", "discovery"};
  RealsenseDiscovery(viam::sdk::Dependencies dependencies,
                     viam::sdk::ResourceConfig configuration,
                     std::shared_ptr<ContextT> ctx)
      : Discovery(configuration.name()), rs_ctx_(std::move(ctx)) {}

  std::vector<viam::sdk::ResourceConfig>
  discover_resources(const viam::sdk::ProtoStruct &extra) override {
    std::vector<viam::sdk::ResourceConfig> configs;

    auto deviceList = rs_ctx_->query_devices();
    int devCount = deviceList.size();

    if (devCount == 0) {
      VIAM_SDK_LOG(warn) << "No Realsense devices found during discovery";
      return {};
    }

    VIAM_SDK_LOG(info) << "Discovered " << devCount << " devices";

    for (auto const &dev : deviceList) {
      if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
        std::string serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        viam::sdk::ProtoStruct attributes;
        attributes.emplace("serial_number", serial_number);

        std::ostringstream name;
        name << "realsense-" << serial_number;

        viam::sdk::ResourceConfig config(
            "camera", std::move(name.str()), "viam", attributes,
            "rdk:component:camera", realsense::Realsense<ContextT>::model,
            viam::sdk::LinkConfig{}, viam::sdk::log_level::info);
        configs.push_back(config);
      }
    }
    return configs;
  }

  viam::sdk::ProtoStruct
  do_command(const viam::sdk::ProtoStruct &command) override {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return viam::sdk::ProtoStruct{};
  }
};

} // namespace discovery
} // namespace realsense