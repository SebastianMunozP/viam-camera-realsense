#pragma once
#include "device.hpp"
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <librealsense2/rs.hpp>

#include <string>
#include <vector>

#include <boost/thread/synchronized_value.hpp>
namespace realsense {

class Realsense;

class RealsenseContext {
public:
    RealsenseContext(std::shared_ptr<rs2::context> ctx) : rs_context_(ctx) {
        setupCallback();
    }
    
    std::shared_ptr<rs2::context> getRsContext() { return rs_context_; }
    
    void addInstance(Realsense* instance) {
        instances_->insert(instance);
        VIAM_SDK_LOG(info) << "[RealsenseContext] Added instance (total: " << instances_->size() << ")";
    }
    
    void removeInstance(Realsense* instance) {
        instances_->erase(instance);
        VIAM_SDK_LOG(info) << "[RealsenseContext] Removed instance (total: " << instances_->size() << ")";
    }

private:
    std::shared_ptr<rs2::context> rs_context_;
    boost::synchronized_value<std::unordered_set<Realsense*>> instances_{std::unordered_set<Realsense*>{}};
    void setupCallback();
    void notifyAllInstances(rs2::event_information& info);
};

// The native config struct for realsense resources.
struct RsResourceConfig {
  std::string resource_name;
  std::string serial_number;

  explicit RsResourceConfig(std::string const &serial_number,
                            std::string const &resource_name)
      : serial_number(serial_number), resource_name(resource_name) {}
};

struct DeviceFunctions {
  std::function<bool(boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>&)> stopDevice;
  std::function<bool(boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>&)> destroyDevice;
  std::function<void(const rs2::device&)> printDeviceInfo;
  std::function<boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>(
      const std::string&, 
      std::shared_ptr<rs2::device>, 
      const std::unordered_set<std::string>&)> createDevice;
  std::function<void(
      const std::string&,
      boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>&,
      boost::synchronized_value<std::shared_ptr<rs2::frameset>>&,
      std::uint64_t)> startDevice;
    std::function<void(
      rs2::event_information&,
      const std::unordered_set<std::string>&,
      boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>&,
      const std::string&,
      boost::synchronized_value<std::shared_ptr<rs2::frameset>>&,
      std::uint64_t)> deviceChangedCallback;
};

class Realsense final : public viam::sdk::Camera,
                        public viam::sdk::Reconfigurable {
public:
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg,
            std::shared_ptr<RealsenseContext> ctx);
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg, std::shared_ptr<RealsenseContext> ctx, DeviceFunctions device_funcs);
  ~Realsense();
  void reconfigure(const viam::sdk::Dependencies &deps,
                   const viam::sdk::ResourceConfig &cfg) override;
  viam::sdk::ProtoStruct
  do_command(const viam::sdk::ProtoStruct &command) override;
  raw_image get_image(std::string mime_type,
                      const viam::sdk::ProtoStruct &extra) override;
  image_collection get_images() override;
  point_cloud get_point_cloud(std::string mime_type,
                              const viam::sdk::ProtoStruct &extra) override;
  properties get_properties() override;
  std::vector<viam::sdk::GeometryConfig>
  get_geometries(const viam::sdk::ProtoStruct &extra) override;

  static std::vector<std::string> validate(viam::sdk::ResourceConfig cfg);
  static viam::sdk::GeometryConfig geometry;
  static viam::sdk::Model model;

  // Handles device changes for this instance
  void handleDeviceChange(rs2::event_information& info);

private:
  boost::synchronized_value<RsResourceConfig> config_;
  boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>> device_;
  boost::synchronized_value<std::shared_ptr<rs2::frameset>> latest_frameset_;

  DeviceFunctions device_funcs_;
  std::shared_ptr<RealsenseContext> realsense_ctx_;

  RsResourceConfig configure(viam::sdk::Dependencies deps,
                             viam::sdk::ResourceConfig cfg);
  static DeviceFunctions createDefaultDeviceFunctions();
};

} // namespace realsense
