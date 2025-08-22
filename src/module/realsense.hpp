#pragma once
#include "device.hpp"
#include "encoding.hpp"
#include "time.hpp"
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <librealsense2/rs.hpp>

#include <string>
#include <vector>
#include <functional>
#include <unordered_set>
#include <memory>

#include <boost/thread/synchronized_value.hpp>
namespace realsense {
static const std::unordered_set<std::string> SUPPORTED_CAMERA_MODELS = {
    "D435", "D435I"};
static constexpr std::uint64_t maxFrameAgeMs =
    1e3; // time until a frame is considered stale, in miliseconds (equal to 1
static constexpr size_t MAX_GRPC_MESSAGE_SIZE =
    33554432; // 32MB gRPC message size limit
const std::string kPcdMimeType = "pointcloud/pcd";
const std::string service_name = "viam_realsense";

template <typename ContextT> class Realsense;

template <typename ContextT> class RealsenseContext {
public:
  RealsenseContext(std::shared_ptr<ContextT> ctx) : rs_context_(ctx) {
    setupCallback();
  }

  std::shared_ptr<ContextT> getRsContext() { return rs_context_; }

  void addInstance(Realsense<ContextT> *instance) {
    instances_->insert(instance);
    VIAM_SDK_LOG(info) << "[RealsenseContext] Added instance (total: "
                       << instances_->size() << ")";
  }

  void removeInstance(Realsense<ContextT> *instance) {
    instances_->erase(instance);
    VIAM_SDK_LOG(info) << "[RealsenseContext] Removed instance (total: "
                       << instances_->size() << ")";
  }

private:
  std::shared_ptr<ContextT> rs_context_;
  boost::synchronized_value<std::unordered_set<Realsense<ContextT> *>>
      instances_{std::unordered_set<Realsense<ContextT> *>{}};

  void setupCallback() {
    rs_context_->set_devices_changed_callback(
        [this](rs2::event_information &info) { notifyAllInstances(info); });
  }

  void notifyAllInstances(rs2::event_information &info) {
    auto instances_guard = instances_.synchronize();
    VIAM_SDK_LOG(info) << "[RealsenseContext] Notifying "
                       << instances_guard->size() << " instances";

    for (Realsense<ContextT> *instance : *instances_guard) {
      if (instance != nullptr) {
        try {
          instance->handleDeviceChange(info);
        } catch (const std::exception &e) {
          VIAM_SDK_LOG(error)
              << "[RealsenseContext] Error notifying instance: " << e.what();
        }
      }
    }
  }
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
  std::function<bool(
      boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>> &)>
      stopDevice;
  std::function<bool(
      boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>> &)>
      destroyDevice;
  std::function<void(const rs2::device &)> printDeviceInfo;
  std::function<
      boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>(
          const std::string &, std::shared_ptr<rs2::device>,
          const std::unordered_set<std::string> &)>
      createDevice;
  std::function<void(
      const std::string &,
      boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>> &,
      boost::synchronized_value<std::shared_ptr<rs2::frameset>> &,
      std::uint64_t)>
      startDevice;
  std::function<void(
      rs2::event_information &, const std::unordered_set<std::string> &,
      boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>> &,
      const std::string &,
      boost::synchronized_value<std::shared_ptr<rs2::frameset>> &,
      std::uint64_t)>
      deviceChangedCallback;
};

template <typename ContextT>
class Realsense final : public viam::sdk::Camera,
                        public viam::sdk::Reconfigurable {
public:
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg,
            std::shared_ptr<RealsenseContext<ContextT>> ctx)
      : Realsense(deps, cfg, ctx, createDefaultDeviceFunctions()) {}
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg,
            std::shared_ptr<RealsenseContext<ContextT>> ctx,
            DeviceFunctions device_funcs)
      : Camera(cfg.name()), config_(configure(deps, cfg)), realsense_ctx_(ctx),
        device_funcs_(device_funcs) {

    std::string requested_serial_number = config_->serial_number;
    VIAM_SDK_LOG(info) << "[constructor] start for resource "
                       << config_->resource_name << " with serial number "
                       << requested_serial_number;

    realsense_ctx_->addInstance(this);
    auto rs_ctx = realsense_ctx_->getRsContext();
    // This will the initial set of connected devices (i.e. the devices that
    // were connected before the callback was set)
    auto deviceList = rs_ctx->query_devices();
    VIAM_SDK_LOG(info) << "[constructor] Amount of connected devices: "
                       << deviceList.size() << "\n";

    for (auto const &dev : deviceList) {
      device_funcs_.printDeviceInfo(dev);

      auto dev_ptr = std::make_shared<rs2::device>(dev);
      std::string connected_device_serial_number =
          dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (requested_serial_number == connected_device_serial_number) {
        device_ = device_funcs_.createDevice(connected_device_serial_number,
                                             dev_ptr, SUPPORTED_CAMERA_MODELS);
        BOOST_ASSERT(device_ != nullptr);

        device_funcs_.startDevice(connected_device_serial_number, device_,
                                  latest_frameset_, maxFrameAgeMs);
        VIAM_SDK_LOG(info) << "[constructor] Device Registered: "
                           << requested_serial_number;
      }
    }

    VIAM_SDK_LOG(info) << "Realsense constructor end "
                       << requested_serial_number;
  }
  ~Realsense() {
    VIAM_SDK_LOG(info) << "Realsense destructor start "
                       << config_->serial_number;
    realsense_ctx_->removeInstance(this);

    device_funcs_.stopDevice(device_);
    device_funcs_.destroyDevice(device_);

    VIAM_SDK_LOG(info) << "Realsense destructor end " << config_->serial_number;
  }
  void reconfigure(const viam::sdk::Dependencies &deps,
                   const viam::sdk::ResourceConfig &cfg) override {
    VIAM_SDK_LOG(info) << "[reconfigure] reconfigure start";
    std::string prev_serial_number;
    prev_serial_number = config_->serial_number;
    VIAM_SDK_LOG(info) << "[reconfigure] stopping device "
                       << prev_serial_number;
    if (not device_funcs_.stopDevice(device_)) {
      VIAM_SDK_LOG(error) << "[reconfigure] failed to stop device "
                          << prev_serial_number;
      throw std::runtime_error("failed to stop device " + prev_serial_number);
    }
    if (not device_funcs_.destroyDevice(device_)) {
      VIAM_SDK_LOG(error) << "[reconfigure] failed to destroy device "
                          << prev_serial_number;
      throw std::runtime_error("failed to destroy device " +
                               prev_serial_number);
    }

    // Before modifying config and starting the new device, let's make sure the
    // new device is actually connected
    config_ = configure(deps, cfg);
    std::string requested_serial_number = config_->serial_number;
    VIAM_SDK_LOG(info) << "[reconfigure] starting device "
                       << requested_serial_number;

    // This will the initial set of connected devices (i.e. the devices that
    // were connected before the callback was set)
    auto deviceList = realsense_ctx_->getRsContext()->query_devices();
    VIAM_SDK_LOG(info) << "[constructor] Number of connected devices: "
                       << deviceList.size() << "\n";

    for (auto const &dev : deviceList) {
      device_funcs_.printDeviceInfo(dev);

      auto dev_ptr = std::make_shared<rs2::device>(dev);
      std::string connected_device_serial_number =
          dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (requested_serial_number == connected_device_serial_number) {
        device_ = device_funcs_.createDevice(connected_device_serial_number,
                                             dev_ptr, SUPPORTED_CAMERA_MODELS);
        BOOST_ASSERT(device_ != nullptr);

        device_funcs_.startDevice(connected_device_serial_number, device_,
                                  latest_frameset_, maxFrameAgeMs);
        VIAM_SDK_LOG(info) << "[reconfigure] Device Registered: "
                           << requested_serial_number;
      }
    }
    VIAM_SDK_LOG(info) << "[reconfigure] Realsense reconfigure end";
  }
  viam::sdk::ProtoStruct
  do_command(const viam::sdk::ProtoStruct &command) override {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return viam::sdk::ProtoStruct();
  }
  viam::sdk::Camera::raw_image
  get_image(std::string mime_type,
            const viam::sdk::ProtoStruct &extra) override {
    try {
      VIAM_SDK_LOG(debug) << "[get_image] start";
      std::string serial_number;
      serial_number = config_->serial_number;
      std::shared_ptr<rs2::frameset> fs = *latest_frameset_;
      if (not fs) {
        VIAM_SDK_LOG(error) << "[get_image] no frameset available";
        throw std::runtime_error("no frameset available");
      }

      BOOST_ASSERT_MSG(fs->get_color_frame(),
                       "[get_image] color frame is invalid");
      time::throwIfTooOld(time::getNowMs(),
                          fs->get_color_frame().get_timestamp(), maxFrameAgeMs,
                          "no recent color frame: check USB connection");

      VIAM_SDK_LOG(debug) << "[get_image] end";
      return encoding::encodeVideoFrameToResponse(fs->get_color_frame());
    } catch (const std::exception &e) {
      VIAM_SDK_LOG(error) << "[get_image] error: " << e.what();
      throw std::runtime_error("failed to create image: " +
                               std::string(e.what()));
    }
  }
  viam::sdk::Camera::image_collection get_images() override {
    try {
      VIAM_SDK_LOG(info) << "[get_images] start";
      std::string serial_number = config_->serial_number;
      std::shared_ptr<rs2::frameset> fs = *latest_frameset_;
      if (not fs) {
        VIAM_SDK_LOG(error) << "[get_images] no frameset available";
        throw std::runtime_error("no frameset available");
      }

      auto color = fs->get_color_frame();
      auto depth = fs->get_depth_frame();

      viam::sdk::Camera::image_collection response;
      response.images.emplace_back(encoding::encodeVideoFrameToResponse(color));
      response.images.emplace_back(encoding::encodeDepthFrameToResponse(depth));

      double colorTS = color.get_timestamp();
      double depthTS = depth.get_timestamp();
      if (colorTS != depthTS) {
        VIAM_SDK_LOG(info)
            << "color and depth timestamps differ, defaulting to "
               "older of the two"
            << "color timestamp was " << colorTS << " depth timestamp was "
            << depthTS;
      }
      // use the older of the two timestamps
      std::uint64_t timestamp =
          static_cast<std::uint64_t>(std::llround(std::min(depthTS, colorTS)));

      std::chrono::microseconds latestTimestamp(timestamp);
      response.metadata.captured_at = viam::sdk::time_pt{
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              latestTimestamp)};
      VIAM_SDK_LOG(info) << "[get_images] end";
      return response;
    } catch (const std::exception &e) {
      VIAM_SDK_LOG(error) << "[get_images] error: " << e.what();
      throw std::runtime_error("failed to create images: " +
                               std::string(e.what()));
    }
  }
  viam::sdk::Camera::point_cloud
  get_point_cloud(std::string mime_type,
                  const viam::sdk::ProtoStruct &extra) override {
    try {
      VIAM_SDK_LOG(debug) << "[get_point_cloud] start";
      std::string serial_number = config_->serial_number;
      std::shared_ptr<rs2::frameset> fs = *latest_frameset_;
      if (not fs) {
        VIAM_SDK_LOG(error) << "[get_point_cloud] no frameset available";
        throw std::runtime_error("no frameset available");
      }

      double nowMs = time::getNowMs();

      rs2::video_frame color_frame = fs->get_color_frame();
      if (not color_frame) {
        throw std::invalid_argument("no color frame");
      }

      time::throwIfTooOld(nowMs, color_frame.get_timestamp(), maxFrameAgeMs,
                          "no recent color frame: check USB connection");

      rs2::depth_frame depth_frame = fs->get_depth_frame();
      if (not depth_frame) {
        throw std::invalid_argument("no depth frame");
      }
      time::throwIfTooOld(nowMs, depth_frame.get_timestamp(), maxFrameAgeMs,
                          "no recent depth frame: check USB connection");

      std::uint8_t *colorData = (std::uint8_t *)color_frame.get_data();
      uint32_t colorDataSize = color_frame.get_data_size();
      if (colorData == nullptr or colorDataSize == 0) {
        throw std::runtime_error("[get_image] color data is null");
      }

      std::uint8_t *depthData = (std::uint8_t *)depth_frame.get_data();
      uint32_t depthDataSize = depth_frame.get_data_size();
      if (depthData == nullptr or depthDataSize == 0) {
        throw std::runtime_error("[get_point_cloud] depth data is null");
      }

      std::shared_ptr<device::ViamRSDevice> my_dev = *device_;
      if (not my_dev) {
        VIAM_SDK_LOG(error) << "[get_point_cloud] no device available";
        throw std::runtime_error("no device available");
      }

      if (not my_dev->started) {
        throw std::runtime_error("device is not started");
      }

      std::vector<std::uint8_t> data = encoding::encodeRGBPointsToPCD(
          my_dev->point_cloud_filter->process(my_dev->align->process(*fs)));

      VIAM_SDK_LOG(debug) << "[get_point_cloud] end";
      if (data.size() > MAX_GRPC_MESSAGE_SIZE) {
        VIAM_SDK_LOG(error)
            << "[get_point_cloud] data size exceeds gRPC message size limit";
        return viam::sdk::Camera::point_cloud{};
      }
      return viam::sdk::Camera::point_cloud{kPcdMimeType, data};
    } catch (const std::exception &e) {
      VIAM_SDK_LOG(error) << "[get_point_cloud] error: " << e.what();
      throw std::runtime_error("failed to create pointcloud: " +
                               std::string(e.what()));
    }
  }
  viam::sdk::Camera::properties get_properties() override {
    try {
      VIAM_SDK_LOG(debug) << "[get_properties] start";
      std::string serial_number = config_->serial_number;
      rs2_intrinsics props;
      std::shared_ptr<device::ViamRSDevice> my_dev = *device_;
      if (not my_dev) {
        VIAM_SDK_LOG(error) << "[get_point_cloud] no device available";
        throw std::runtime_error("no device available");
      }
      if (not my_dev->started or not my_dev->pipe) {
        std::ostringstream buffer;
        buffer << service_name << ": device with serial number "
               << serial_number << " is not longer started";
        throw std::invalid_argument(buffer.str());
      }
      auto color_stream = my_dev->pipe->get_active_profile()
                              .get_stream(RS2_STREAM_COLOR)
                              .as<rs2::video_stream_profile>();
      if (not color_stream) {
        throw std::runtime_error("color stream is not available");
      }
      props = color_stream.get_intrinsics();

      viam::sdk::Camera::properties p{};
      p.supports_pcd = true;
      p.intrinsic_parameters.width_px = props.width;
      p.intrinsic_parameters.height_px = props.height;
      p.intrinsic_parameters.focal_x_px = props.fx;
      p.intrinsic_parameters.focal_y_px = props.fy;
      p.intrinsic_parameters.center_x_px = props.ppx;
      p.intrinsic_parameters.center_y_px = props.ppy;
      p.distortion_parameters.model = rs2_distortion_to_string(props.model);
      for (auto const &coeff : props.coeffs)
        p.distortion_parameters.parameters.push_back(coeff);

      std::stringstream coeffs_stream;
      for (size_t i = 0; i < p.distortion_parameters.parameters.size(); ++i) {
        if (i > 0)
          coeffs_stream << ", ";
        coeffs_stream << p.distortion_parameters.parameters[i];
      }

      VIAM_SDK_LOG(debug)
          << "[get_properties] properties: ["
          << "width: " << p.intrinsic_parameters.width_px << ", "
          << "height: " << p.intrinsic_parameters.height_px << ", "
          << "focal_x: " << p.intrinsic_parameters.focal_x_px << ", "
          << "focal_y: " << p.intrinsic_parameters.focal_y_px << ", "
          << "center_x: " << p.intrinsic_parameters.center_x_px << ", "
          << "center_y: " << p.intrinsic_parameters.center_y_px << ", "
          << "distortion_model: " << p.distortion_parameters.model << ", "
          << "distortion_coeffs: [" << coeffs_stream.str() << "]" << "]";

      VIAM_SDK_LOG(debug) << "[get_properties] end";
      return p;
    } catch (const std::exception &e) {
      VIAM_SDK_LOG(error) << "[get_properties] error: " << e.what();
      throw std::runtime_error("failed to create properties: " +
                               std::string(e.what()));
    }
  }
  std::vector<viam::sdk::GeometryConfig>
  get_geometries(const viam::sdk::ProtoStruct &extra) override {
    // This is the geometry for the D435 and D435i, the only models that we
    // currently support. See
    // https://github.com/viam-modules/viam-camera-realsense/pull/75 for
    // explanation of values. NOTE: If support for additional RealSense camera
    // models is added, update method accordingly.
    return {viam::sdk::GeometryConfig(viam::sdk::pose{-17.5, 0, -12.5},
                                      viam::sdk::box({90, 25, 25}), "box")};
  }

  static std::vector<std::string> validate(viam::sdk::ResourceConfig cfg) {
    VIAM_SDK_LOG(info) << "[validate] Validating that serial_number is present";
    auto attrs = cfg.attributes();

    if (not attrs.count("serial_number")) {
      VIAM_SDK_LOG(error)
          << "[validate] Missing required attribute: serial_number";
      throw std::invalid_argument("serial_number is a required argument");
    }

    VIAM_SDK_LOG(info)
        << "[validate] Validating that serial_number is a string";
    if (not attrs["serial_number"].is_a<std::string>()) {
      VIAM_SDK_LOG(error) << "[validate] serial_number is not a string";
      throw std::invalid_argument("serial_number must be a string");
    }

    VIAM_SDK_LOG(info)
        << "[validate] Validating that serial_number is not empty";
    // We already stablished this is a string, so it's safe to call this
    std::string const serial =
        attrs["serial_number"].get_unchecked<std::string>();
    if (serial.empty()) {
      VIAM_SDK_LOG(error) << "[validate] serial_number is empty";
      throw std::invalid_argument("serial_number must be a non-empty string");
    }
    // If we reach here, the serial number is valid
    return {};
  }
  static inline viam::sdk::Model model{"viam", "camera", "realsense"};

  // Handles device changes for this instance
  void handleDeviceChange(rs2::event_information &info) {
    VIAM_SDK_LOG(info) << "[handleDeviceChange] Processing for serial: "
                       << config_->serial_number;

    device_funcs_.deviceChangedCallback(info, SUPPORTED_CAMERA_MODELS, device_,
                                        config_->serial_number,
                                        latest_frameset_, maxFrameAgeMs);
  }

private:
  boost::synchronized_value<RsResourceConfig> config_;
  boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>> device_;
  boost::synchronized_value<std::shared_ptr<rs2::frameset>> latest_frameset_;

  DeviceFunctions device_funcs_;
  std::shared_ptr<RealsenseContext<ContextT>> realsense_ctx_;

  RsResourceConfig configure(viam::sdk::Dependencies dependencies,
                             viam::sdk::ResourceConfig configuration) {
    auto attrs = configuration.attributes();

    /*
     * Validation already checks that serial_number exists, it is a string and
     * it is not empty, so we can safely extract it without additional checks.
     */
    std::string const serial =
        attrs["serial_number"].get_unchecked<std::string>();
    auto native_config =
        realsense::RsResourceConfig(serial, configuration.name());

    return native_config;
  }
  static DeviceFunctions createDefaultDeviceFunctions() {
    return DeviceFunctions{
        .stopDevice =
            [](boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>
                   &device) { return device::stopDevice(device); },
        .destroyDevice =
            [](boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>
                   &device) { return device::destroyDevice(device); },
        .printDeviceInfo =
            [](const rs2::device &dev) { device::printDeviceInfo(dev); },
        .createDevice =
            [](const std::string &serial, std::shared_ptr<rs2::device> dev_ptr,
               const std::unordered_set<std::string> &supported_models) {
              return device::createDevice(serial, dev_ptr, supported_models);
            },
        .startDevice =
            [](const std::string &serial,
               boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>
                   &device,
               boost::synchronized_value<std::shared_ptr<rs2::frameset>>
                   &latest_frameset,
               std::uint64_t maxFrameAgeMs) {
              return device::startDevice(serial, device, latest_frameset,
                                         maxFrameAgeMs);
            },
        .deviceChangedCallback =
            [](rs2::event_information &info,
               const std::unordered_set<std::string> &supported_models,
               boost::synchronized_value<std::shared_ptr<device::ViamRSDevice>>
                   &device,
               const std::string &serial,
               boost::synchronized_value<std::shared_ptr<rs2::frameset>>
                   &latest_frameset,
               std::uint64_t maxFrameAgeMs) {
              device::deviceChangedCallback(info, supported_models, device,
                                            serial, latest_frameset,
                                            maxFrameAgeMs);
            }};
  }
};
} // namespace realsense
