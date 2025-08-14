
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

// GLOBALS BEGIN
// Using the "Construct on First Use Idiom" to prevent the static
// deinitialization order fiasco. See
// https://isocpp.org/wiki/faq/ctors#static-init-order
//
// These functions wrap the static objects, ensuring they are constructed only
// when first accessed and correctly destructed to prevent the non-deterministic
// deinitialization such as the one that caused:
// https://viam.atlassian.net/browse/RSDK-11170
std::mutex &serial_by_resource_mu() {
  static std::mutex mu;
  return mu;
}

std::unordered_map<std::string, std::string> &serial_by_resource() {
  static std::unordered_map<std::string, std::string> devices;
  return devices;
}

std::mutex &devices_by_serial_mu() {
  static std::mutex mu;
  return mu;
}

std::unordered_map<std::string, std::shared_ptr<device::ViamRSDevice>> &
devices_by_serial() {
  static std::unordered_map<std::string, std::shared_ptr<device::ViamRSDevice>>
      devices;
  return devices;
}

std::mutex &frame_set_by_serial_mu() {
  static std::mutex mu;
  return mu;
}

std::unordered_map<std::string, std::shared_ptr<rs2::frameset>> &
frame_set_by_serial() {
  static std::unordered_map<std::string, std::shared_ptr<rs2::frameset>>
      frame_sets;
  return frame_sets;
}

std::mutex &config_mu_() {
  static std::mutex mu;
  return mu;
}

// GLOBALS END

// HELPERS BEGIN
//
std::shared_ptr<rs2::frameset>
getFramesetBySerial(const std::string &serial_number) {
  std::shared_ptr<rs2::frameset> fs = nullptr;
  {
    std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
    auto search = frame_set_by_serial().find(serial_number);
    if (search == frame_set_by_serial().end()) {
      throw std::invalid_argument("no frame yet");
    }
    fs = search->second;
  }
  return fs;
}

std::string Realsense::getSerialNumber() {
  std::string serial_number;
  {
    const std::lock_guard<std::mutex> lock(config_mu_());
    serial_number = config_->serial_number;
  }
  return serial_number;
}

std::shared_ptr<device::ViamRSDevice>
getDeviceBySerial(std::string const &serial_number) {
  std::lock_guard<std::mutex> lock(devices_by_serial_mu());
  auto search = devices_by_serial().find(serial_number);
  if (search == devices_by_serial().end()) {
    throw std::invalid_argument("Device not found: " + serial_number);
  }
  return search->second;
}

// HELPERS END

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

Realsense::Realsense(vsdk::Dependencies deps, vsdk::ResourceConfig cfg)
    : Camera(cfg.name()), config_(configure_(std::move(deps), std::move(cfg))) {
  VIAM_SDK_LOG(info) << "Realsense constructor start "
                     << config_->serial_number;
  device::startDevice(
      config_->serial_number, getDeviceBySerial(config_->serial_number),
      frame_set_by_serial_mu(), frame_set_by_serial(), maxFrameAgeMs);
  {
    std::lock_guard<std::mutex> lock(serial_by_resource_mu());
    serial_by_resource()[config_->resource_name] = config_->serial_number;
  }
  VIAM_SDK_LOG(info) << "Realsense constructor end " << config_->serial_number;
}
Realsense::~Realsense() {
  if (config_ == nullptr) {
    VIAM_SDK_LOG(error) << "Realsense destructor start: config_ is null, no "
                           "available serial number";
  } else {
    VIAM_SDK_LOG(info) << "Realsense destructor start "
                       << config_->serial_number;
  }
  std::string prev_serial_number;
  std::string prev_resource_name;
  {
    const std::lock_guard<std::mutex> lock(config_mu_());
    prev_serial_number = config_->serial_number;
    prev_resource_name = config_->resource_name;
  }
  device::stopDevice(prev_serial_number, prev_resource_name,
                     getDeviceBySerial(prev_serial_number),
                     serial_by_resource_mu(), serial_by_resource());
  if (config_ == nullptr) {
    VIAM_SDK_LOG(error) << "Realsense destructor end: config_ is null, no "
                           "available serial number";
  } else {
    VIAM_SDK_LOG(info) << "Realsense destructor end " << config_->serial_number;
  }
}

void Realsense::reconfigure(const viam::sdk::Dependencies &deps,
                            const viam::sdk::ResourceConfig &cfg) {
  VIAM_SDK_LOG(info) << "[reconfigure] reconfigure start";
  std::string prev_serial_number;
  std::string prev_resource_name;
  {
    const std::lock_guard<std::mutex> lock(config_mu_());
    prev_serial_number = config_->serial_number;
    prev_resource_name = config_->resource_name;
  }
  VIAM_SDK_LOG(info) << "[reconfigure] stopping device " << prev_serial_number;
  device::stopDevice(prev_serial_number, prev_resource_name,
                     getDeviceBySerial(prev_serial_number),
                     serial_by_resource_mu(), serial_by_resource());

  // Before modifying config and starting the new device, let's make sure the
  // new device is actually connected
  auto temp_config = configure_(deps, cfg);
  if (not device::isDeviceConnected(temp_config->serial_number)) {
    VIAM_SDK_LOG(error) << "[reconfigure] new device is not connected";
    return;
  }

  std::string new_serial_number;
  std::string new_resource_name;
  {
    const std::lock_guard<std::mutex> lock(config_mu_());
    config_.reset();
    config_ = std::move(temp_config);
    new_serial_number = config_->serial_number;
    new_resource_name = config_->resource_name;
  }
  VIAM_SDK_LOG(info) << "[reconfigure] starting device " << new_serial_number;
  device::startDevice(
      new_serial_number, getDeviceBySerial(config_->serial_number),
      frame_set_by_serial_mu(), frame_set_by_serial(), maxFrameAgeMs);
  {
    std::lock_guard<std::mutex> lock(serial_by_resource_mu());
    serial_by_resource()[config_->resource_name] = new_serial_number;
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
    VIAM_SDK_LOG(info) << "[get_image] start";
    std::string serial_number;
    {
      const std::lock_guard<std::mutex> lock(config_mu_());
      serial_number = config_->serial_number;
    }
    std::shared_ptr<rs2::frameset> fs = nullptr;
    {
      std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
      auto search = frame_set_by_serial().find(serial_number);
      if (search == frame_set_by_serial().end()) {
        throw std::invalid_argument("no frame yet");
      }
      fs = search->second;
    }
    VIAM_SDK_LOG(info) << "[get_image] end";
    BOOST_ASSERT_MSG(fs->get_color_frame(),
                     "[encodeFrameToResponse] color frame is invalid");
    time::throwIfTooOld(time::getNowMs(), fs->get_color_frame().get_timestamp(),
                        maxFrameAgeMs,
                        "no recent color frame: check USB connection");
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
    std::string serial_number = getSerialNumber();
    std::shared_ptr<rs2::frameset> fs = getFramesetBySerial(serial_number);

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
    VIAM_SDK_LOG(info) << "[get_point_cloud] start";
    std::string serial_number = getSerialNumber();
    std::shared_ptr<rs2::frameset> fs = getFramesetBySerial(serial_number);
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

    std::shared_ptr<device::ViamRSDevice> my_dev =
        getDeviceBySerial(serial_number);
    if (not my_dev->started) {
      throw std::runtime_error("device is not started");
    }

    std::vector<std::uint8_t> data = encoding::encodeRGBPointsToPCD(
        my_dev->point_cloud_filter->process(my_dev->align->process(*fs)));

    VIAM_SDK_LOG(info) << "[get_point_cloud] end";
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
    VIAM_SDK_LOG(info) << "[get_properties] start";
    std::string serial_number = getSerialNumber();
    rs2_intrinsics props;
    std::shared_ptr<device::ViamRSDevice> my_dev =
        getDeviceBySerial(serial_number);
    if (not my_dev or not my_dev->started or not my_dev->pipe) {
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

    VIAM_SDK_LOG(info) << "[get_properties] properties: ["
                       << "width: " << p.intrinsic_parameters.width_px << ", "
                       << "height: " << p.intrinsic_parameters.height_px << ", "
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

    VIAM_SDK_LOG(info) << "[get_properties] end";
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

std::unique_ptr<RsResourceConfig>
Realsense::configure_(vsdk::Dependencies dependencies,
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

  auto native_config = std::make_unique<realsense::RsResourceConfig>(
      serial_number_from_config, configuration.name());

  return native_config;
}

void startRealsenseSDK(std::shared_ptr<rs2::context> ctx) {
  ctx->set_devices_changed_callback([](rs2::event_information &info) {
    device::deviceChangedCallback(
        info, SUPPORTED_CAMERA_MODELS, devices_by_serial_mu(),
        devices_by_serial(), serial_by_resource_mu(), serial_by_resource(),
        frame_set_by_serial_mu(), frame_set_by_serial(), maxFrameAgeMs);
  });
  // This will the initial set of connected devices (i.e. the devices that were
  // connected before the callback was set)
  auto deviceList = ctx->query_devices();
  VIAM_SDK_LOG(info) << "devCount: " << deviceList.size() << "\n";

  for (auto const &dev : deviceList) {
    auto dev_ptr = std::make_shared<rs2::device>(dev);
    device::printDeviceInfo(*dev_ptr);
    device::registerDevice(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), dev_ptr,
                           SUPPORTED_CAMERA_MODELS, devices_by_serial_mu(),
                           devices_by_serial());
    VIAM_SDK_LOG(info) << "Device Added: "
                       << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  }
  VIAM_SDK_LOG(info) << "Realsense SDK started, devices_by_serial size: "
                     << devices_by_serial().size();
}

} // namespace realsense
