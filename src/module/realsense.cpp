
#include "realsense.hpp"
#include "device.hpp"
#include "encoding.hpp"
#include "time.hpp"
#include <fstream>
#include <iostream>
#include <optional>
#include <unordered_set>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/rpc/server.hpp>
#include <viam/sdk/spatialmath/geometry.hpp>

namespace realsense {

namespace vsdk = ::viam::sdk;

vsdk::Model Realsense::model("viam", "camera", "realsense");

// CONSTANTS BEGIN
const std::string kResourceType = "CameraRealSense";
const std::string kAPINamespace = "viam";
const std::string kAPIType = "camera";
const std::string kAPISubtype = "realsense";

const std::string kColorSourceName = "color";
const std::string kColorMimeTypeJPEG = "image/jpeg";
const std::string kDepthSourceName = "depth";
const std::string kDepthMimeTypeViamDep = "image/vnd.viam.dep";
const std::string kPcdMimeType = "pointcloud/pcd";

const std::string service_name = "viam_realsense";
static constexpr std::uint64_t maxFrameAgeMs =
    1e3; // time until a frame is considered stale, in miliseconds (equal to 1
         // sec)

static constexpr size_t MAX_GRPC_MESSAGE_SIZE =
    33554432; // 32MB gRPC message size limit
static const std::unordered_set<std::string> SUPPORTED_CAMERA_MODELS = {
    "D435", "D435I"};

// CONSTANTS END

// RESOURCE BEGIN
std::vector<std::string> validate(vsdk::ResourceConfig cfg) {
  auto attrs = cfg.attributes();

  if (attrs.count("serial_number")) {
    if (!attrs["serial_number"].get<std::string>()) {
      throw std::invalid_argument("serial_number must be a string");
    }
  }
  return {};
}

Realsense::Realsense(vsdk::Dependencies deps, vsdk::ResourceConfig cfg, std::shared_ptr<rs2::context> ctx)
    : Camera(cfg.name()), config_(configure(std::move(deps), std::move(cfg))), ctx_(ctx) {

  std::string requested_serial_number = config_->serial_number;
  VIAM_SDK_LOG(info) << "[constructor] start " << requested_serial_number;

  ctx_->set_devices_changed_callback([this](rs2::event_information &info) {
    device::deviceChangedCallback(info, SUPPORTED_CAMERA_MODELS, device_,
                                  config_->serial_number, latest_frameset_,
                                  maxFrameAgeMs);
  });
  // This will the initial set of connected devices (i.e. the devices that were
  // connected before the callback was set)
  auto deviceList = ctx_->query_devices();
  VIAM_SDK_LOG(info) << "[constructor] Amount of connected devices: "
                     << deviceList.size() << "\n";

  for (auto const &dev : deviceList) {
    device::printDeviceInfo(dev);

    auto dev_ptr = std::make_shared<rs2::device>(dev);
    std::string connected_device_serial_number =
        dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (requested_serial_number == connected_device_serial_number) {
      device_ = device::createDevice(connected_device_serial_number, dev_ptr,
                                     SUPPORTED_CAMERA_MODELS);
      BOOST_ASSERT(device_ != nullptr);

      device::startDevice(connected_device_serial_number, device_,
                          latest_frameset_, maxFrameAgeMs);
      VIAM_SDK_LOG(info) << "[constructor] Device Registered: "
                         << requested_serial_number;
    }
  }

  VIAM_SDK_LOG(info) << "Realsense constructor end " << requested_serial_number;
}
Realsense::~Realsense() {
  VIAM_SDK_LOG(info) << "Realsense destructor start " << config_->serial_number;
  std::string prev_serial_number;
  std::string prev_resource_name;
  prev_serial_number = config_->serial_number;
  prev_resource_name = config_->resource_name;
  device::stopDevice(device_);
  device::destroyDevice(device_);

  VIAM_SDK_LOG(info) << "Realsense destructor end " << config_->serial_number;
}

void Realsense::reconfigure(const viam::sdk::Dependencies &deps,
                            const viam::sdk::ResourceConfig &cfg) {
  VIAM_SDK_LOG(info) << "[reconfigure] reconfigure start";
  std::string prev_serial_number;
  prev_serial_number = config_->serial_number;
  VIAM_SDK_LOG(info) << "[reconfigure] stopping device " << prev_serial_number;
  device::stopDevice(device_);
  device::destroyDevice(device_);

  // Before modifying config and starting the new device, let's make sure the
  // new device is actually connected
  config_ = configure(deps, cfg);
  std::string requested_serial_number = config_->serial_number;
  VIAM_SDK_LOG(info) << "[reconfigure] starting device "
                     << requested_serial_number;

  // This will the initial set of connected devices (i.e. the devices that were
  // connected before the callback was set)
  auto deviceList = ctx_->query_devices();
  VIAM_SDK_LOG(info) << "[constructor] Amount of connected devices: "
                     << deviceList.size() << "\n";

  for (auto const &dev : deviceList) {
    device::printDeviceInfo(dev);

    auto dev_ptr = std::make_shared<rs2::device>(dev);
    std::string connected_device_serial_number =
        dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (requested_serial_number == connected_device_serial_number) {
      device_ = device::createDevice(connected_device_serial_number, dev_ptr,
                                     SUPPORTED_CAMERA_MODELS);
      BOOST_ASSERT(device_ != nullptr);

      device::startDevice(connected_device_serial_number, device_,
                          latest_frameset_, maxFrameAgeMs);
      VIAM_SDK_LOG(info) << "[reconfigure] Device Registered: "
                         << requested_serial_number;
    }
  }
  VIAM_SDK_LOG(info) << "[reconfigure] Realsense reconfigure end";
}
viam::sdk::ProtoStruct
Realsense::do_command(const viam::sdk::ProtoStruct &command) {
  VIAM_SDK_LOG(error) << "do_command not implemented";
  return viam::sdk::ProtoStruct();
}
viam::sdk::Camera::raw_image
Realsense::get_image(std::string mime_type,
                     const viam::sdk::ProtoStruct &extra) {
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
                     "[encodeFrameToResponse] color frame is invalid");
    time::throwIfTooOld(time::getNowMs(), fs->get_color_frame().get_timestamp(),
                        maxFrameAgeMs,
                        "no recent color frame: check USB connection");

    VIAM_SDK_LOG(debug) << "[get_image] end";
    return encoding::encodeFrameToResponse(fs->get_color_frame());
  } catch (const std::exception &e) {
    VIAM_SDK_LOG(error) << "[get_image] error: " << e.what();
    throw std::runtime_error("failed to create image: " +
                             std::string(e.what()));
  }
}
viam::sdk::Camera::image_collection Realsense::get_images() {
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
    response.images.emplace_back(encoding::encodeFrameToResponse(color));
    response.images.emplace_back(encoding::encodeFrameToResponse(depth));

    double colorTS = color.get_timestamp();
    double depthTS = depth.get_timestamp();
    if (colorTS != depthTS) {
      VIAM_SDK_LOG(info) << "color and depth timestamps differ, defaulting to "
                            "older of the two"
                         << "color timestamp was " << colorTS
                         << " depth timestamp was " << depthTS;
    }
    // use the older of the two timestamps
    uint64_t timestamp = (depthTS < colorTS) ? depthTS : colorTS;

    std::chrono::microseconds latestTimestamp(timestamp);
    response.metadata.captured_at = vsdk::time_pt{
        std::chrono::duration_cast<std::chrono::nanoseconds>(latestTimestamp)};
    VIAM_SDK_LOG(info) << "[get_images] end";
    return response;
  } catch (const std::exception &e) {
    VIAM_SDK_LOG(error) << "[get_images] error: " << e.what();
    throw std::runtime_error("failed to create images: " +
                             std::string(e.what()));
  }
}
viam::sdk::Camera::point_cloud
Realsense::get_point_cloud(std::string mime_type,
                           const viam::sdk::ProtoStruct &extra) {
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
      return vsdk::Camera::point_cloud{};
    }
    return vsdk::Camera::point_cloud{kPcdMimeType, data};
  } catch (const std::exception &e) {
    VIAM_SDK_LOG(error) << "[get_point_cloud] error: " << e.what();
    throw std::runtime_error("failed to create pointcloud: " +
                             std::string(e.what()));
  }
}
viam::sdk::Camera::properties Realsense::get_properties() {
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
      buffer << service_name << ": device with serial number " << serial_number
             << " is not longer started";
      throw std::invalid_argument(buffer.str());
    }
    auto color_stream = my_dev->pipe->get_active_profile()
                            .get_stream(RS2_STREAM_COLOR)
                            .as<rs2::video_stream_profile>();
    if (not color_stream) {
      throw std::runtime_error("color stream is not available");
    }
    props = color_stream.get_intrinsics();

    vsdk::Camera::properties p{};
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

    VIAM_SDK_LOG(debug) << "[get_properties] properties: ["
                        << "width: " << p.intrinsic_parameters.width_px << ", "
                        << "height: " << p.intrinsic_parameters.height_px
                        << ", "
                        << "focal_x: " << p.intrinsic_parameters.focal_x_px
                        << ", "
                        << "focal_y: " << p.intrinsic_parameters.focal_y_px
                        << ", "
                        << "center_x: " << p.intrinsic_parameters.center_x_px
                        << ", "
                        << "center_y: " << p.intrinsic_parameters.center_y_px
                        << ", "
                        << "distortion_model: " << p.distortion_parameters.model
                        << ", " << "distortion_coeffs: [" << coeffs_stream.str()
                        << "]" << "]";

    VIAM_SDK_LOG(debug) << "[get_properties] end";
    return p;
  } catch (const std::exception &e) {
    VIAM_SDK_LOG(error) << "[get_properties] error: " << e.what();
    throw std::runtime_error("failed to create properties: " +
                             std::string(e.what()));
  }
}
std::vector<viam::sdk::GeometryConfig>
Realsense::get_geometries(const viam::sdk::ProtoStruct &extra) {
  // This is the geometry for the D435 and D435i, the only models that we
  // currently support. See
  // https://github.com/viam-modules/viam-camera-realsense/pull/75 for
  // explanation of values. NOTE: If support for additional RealSense camera
  // models is added, update method accordingly.
  return {vsdk::GeometryConfig(vsdk::pose{-17.5, 0, -12.5},
                               vsdk::box({90, 25, 25}), "box")};
}

RsResourceConfig Realsense::configure(vsdk::Dependencies dependencies,
                                      vsdk::ResourceConfig configuration) {
  auto attrs = configuration.attributes();

  std::string serial_number_from_config;
  if (!attrs.count("serial_number")) {
    throw std::invalid_argument("serial_number is a required argument");
  }

  const std::string *serial_val = attrs["serial_number"].get<std::string>();
  if (serial_val == nullptr) {
    throw std::invalid_argument("serial_number must be a string");
  }

  serial_number_from_config = *serial_val;

  auto native_config = realsense::RsResourceConfig(serial_number_from_config,
                                                   configuration.name());

  return native_config;
}

} // namespace realsense
