
#include "realsense.hpp"
#include "encoding.hpp"
#include "utils.hpp"
#include <fstream>
#include <iostream>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/rpc/server.hpp>

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

constexpr std::string_view service_name = "viam_realsense";
const float mmToMeterMultiple = 0.001;
static const double min_distance = 1e-6;
static constexpr std::uint64_t maxFrameAgeMs =
    1e3; // time until a frame is considered stale, in miliseconds (equal to 1
         // sec)

static constexpr size_t MAX_GRPC_MESSAGE_SIZE =
    33554432; // 32MB gRPC message size limit

// CONSTANTS END

// STRUCTS BEGIN
class PointCloudFilter {
public:
  PointCloudFilter() : pointcloud_(std::make_shared<rs2::pointcloud>()) {}
  std::pair<rs2::points, rs2::video_frame> process(rs2::frameset frameset) {
    auto depth_frame = frameset.get_depth_frame();
    if (!depth_frame) {
      throw std::runtime_error("No depth frame in frameset");
    }
    auto color_frame = frameset.get_color_frame();
    if (!color_frame) {
      throw std::runtime_error("No color frame in frameset");
    }
    pointcloud_->map_to(color_frame);
    auto points = pointcloud_->calculate(depth_frame);
    return std::make_pair(points, color_frame);
  }

private:
  std::shared_ptr<rs2::pointcloud> pointcloud_;
};

struct ViamRSDevice {
  ~ViamRSDevice() {
    std::cerr << "deleting ViamRSDevice " << serial_number << std::endl;
  }
  std::string serial_number;
  std::shared_ptr<rs2::device> device;
  bool started;
  std::shared_ptr<rs2::pipeline> pipe;
  std::shared_ptr<PointCloudFilter> point_cloud_filter;
  std::shared_ptr<rs2::align> align;
  std::shared_ptr<rs2::config> config;
  size_t frame_count = 0;
  std::chrono::steady_clock::time_point last_report =
      std::chrono::steady_clock::now();
};

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

std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>> &
devices_by_serial() {
  static std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>> devices;
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

std::shared_ptr<ViamRSDevice>
getDeviceBySerial(std::string const &serial_number) {
  std::lock_guard<std::mutex> lock(devices_by_serial_mu());
  auto search = devices_by_serial().find(serial_number);
  if (search == devices_by_serial().end()) {
    throw std::invalid_argument("Device not found: " + serial_number);
  }
  return search->second;
}

void printDeviceInfo(rs2::device const &dev) {
  std::cout << "DeviceInfo:\n"
            << "  Name:                           "
            << dev.get_info(RS2_CAMERA_INFO_NAME) << "\n"
            << "  Serial Number:                  "
            << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\n"
            << "  Firmware Version:               "
            << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n"
            << "  Recommended Firmware Version:   "
            << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)
            << "\n"
            << "  ASIC Serial Number:             "
            << dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << "\n"
            << "  USB Type Descriptor:            "
            << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << "\n";
}
void printDeviceList(const std::shared_ptr<rs2::device_list> devList) {
  std::for_each(devList->begin(), devList->end(),
                [](const rs2::device &dev) { printDeviceInfo(dev); });
}

void startDevice(std::string serialNumber, std::string resourceName) {
  VIAM_SDK_LOG(info) << service_name << ": starting device " << serialNumber;
  std::shared_ptr<ViamRSDevice> my_dev = getDeviceBySerial(serialNumber);
  if (my_dev->started) {
    std::ostringstream buffer;
    buffer << service_name << ": unable to start already started device "
           << serialNumber;
    throw std::invalid_argument(buffer.str());
  }

  auto frameCallback = [serialNumber](rs2::frame const &frame) {
    if (frame.is<rs2::frameset>()) {
      // With callbacks, all synchronized stream will arrive in a single
      // frameset
      auto frameset =
          std::make_shared<rs2::frameset>(frame.as<rs2::frameset>());
      if (frameset->size() != 2) {
        std::cerr << "got non 2 frame count: " << frameset->size() << std::endl;
        return;
      }
      auto color_frame = frameset->get_color_frame();
      if (not color_frame) {
        std::cerr << "no color frame" << std::endl;
        return;
      }

      auto depth_frame = frameset->get_depth_frame();
      if (not depth_frame) {
        std::cerr << "no depth frame" << std::endl;
        return;
      }

      std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
      double nowMs = utils::getNowMs();
      utils::logIfTooOld(std::cerr, nowMs, color_frame.get_timestamp(),
                         maxFrameAgeMs,
                         "[frame_callback] received color frame is too stale");
      utils::logIfTooOld(std::cerr, nowMs, depth_frame.get_timestamp(),
                         maxFrameAgeMs,
                         "[frame_callback] received depth frame is too stale");

      auto it = frame_set_by_serial().find(serialNumber);
      if (it != frame_set_by_serial().end()) {
        rs2::frame prevColor = it->second->get_color_frame();
        rs2::frame prevDepth = it->second->get_depth_frame();
        if (prevColor and prevDepth) {
          utils::logIfTooOld(
              std::cerr, color_frame.get_timestamp(), prevColor.get_timestamp(),
              maxFrameAgeMs,
              "[frameCallback] previous color frame is too stale");
          utils::logIfTooOld(
              std::cerr, depth_frame.get_timestamp(), prevDepth.get_timestamp(),
              maxFrameAgeMs,
              "[frameCallback] previous depth frame is too stale");
        }
      }
      frame_set_by_serial()[serialNumber] =
          std::make_shared<rs2::frameset>(frame.as<rs2::frameset>());
    } else {
      // Stream that bypass synchronization (such as IMU) will produce single
      // frames
      std::cerr << "got non 2 a frameset: " << frame.get_profile().stream_name()
                << std::endl;
      return;
    }
  };

  my_dev->pipe->start(*my_dev->config, std::move(frameCallback));
  my_dev->started = true;
  VIAM_SDK_LOG(info) << service_name << ": device started " << serialNumber;
}

void stopDevice(std::string serialNumber, std::string resourceName) {
  std::shared_ptr<ViamRSDevice> my_dev = getDeviceBySerial(serialNumber);
  if (not my_dev or not my_dev->started) {
    VIAM_SDK_LOG(error)
        << service_name
        << ": unable to stop device that is not currently running "
        << serialNumber;
    return;
  }

  my_dev->pipe->stop();
  my_dev->started = false;
  {
    std::lock_guard<std::mutex> lock(serial_by_resource_mu());
    serial_by_resource().erase(resourceName);
  }
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
  startDevice(config_->serial_number, config_->resource_name);
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
  stopDevice(prev_serial_number, prev_resource_name);
  if (config_ == nullptr) {
    VIAM_SDK_LOG(error) << "Realsense destructor end: config_ is null, no "
                           "available serial number";
  } else {
    VIAM_SDK_LOG(info) << "Realsense destructor end " << config_->serial_number;
  }
}
void Realsense::reconfigure(const viam::sdk::Dependencies &deps,
                            const viam::sdk::ResourceConfig &cfg) {}
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
    utils::throwIfTooOld(utils::getNowMs(),
                         fs->get_color_frame().get_timestamp(), maxFrameAgeMs,
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
    double nowMs = utils::getNowMs();

    rs2::video_frame color_frame = fs->get_color_frame();
    if (not color_frame) {
      throw std::invalid_argument("no color frame");
    }

    utils::throwIfTooOld(nowMs, color_frame.get_timestamp(), maxFrameAgeMs,
                         "no recent color frame: check USB connection");

    rs2::depth_frame depth_frame = fs->get_depth_frame();
    if (not depth_frame) {
      throw std::invalid_argument("no depth frame");
    }
    utils::throwIfTooOld(nowMs, depth_frame.get_timestamp(), maxFrameAgeMs,
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

    std::shared_ptr<ViamRSDevice> my_dev = getDeviceBySerial(serial_number);
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
    std::shared_ptr<ViamRSDevice> my_dev = getDeviceBySerial(serial_number);
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
  return std::vector<viam::sdk::GeometryConfig>();
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

// REALSENSE SDK DEVICE REGISTRY START
bool checkIfMatchingColorDepthProfiles(const rs2::video_stream_profile &color,
                                       const rs2::video_stream_profile &depth) {
  if (std::tuple(color.width(), color.height(), color.fps()) ==
      std::tuple(depth.width(), depth.height(), depth.fps())) {
    std::cout << "using width: " << color.width()
              << " height: " << color.height() << " fps: " << color.fps()
              << "\n";
    return true;
  }
  return false;
}

// create a config for software depth-to-color alignment
std::shared_ptr<rs2::config>
createSwD2CAlignConfig(std::shared_ptr<rs2::pipeline> pipe,
                       std::shared_ptr<rs2::device> dev) {
  auto cfg = std::make_shared<rs2::config>();

  // Query all sensors for the device
  std::vector<rs2::sensor> sensors = dev->query_sensors();
  rs2::sensor color_sensor, depth_sensor;
  for (auto &s : sensors) {
    if (s.is<rs2::color_sensor>())
      color_sensor = s;
    if (s.is<rs2::depth_sensor>())
      depth_sensor = s;
  }

  // Get stream profiles
  auto color_profiles = color_sensor.get_stream_profiles();
  VIAM_SDK_LOG(info) << "Found " << color_profiles.size() << " color profiles";
  // Log color profiles
  VIAM_SDK_LOG(info) << "Available color profiles:";
  for (auto &cp : color_profiles) {
    auto csp = cp.as<rs2::video_stream_profile>();
    VIAM_SDK_LOG(info) << "Color profile: " << csp.stream_name()
                       << " format: " << csp.format()
                       << " width: " << csp.width()
                       << " height: " << csp.height() << " fps: " << csp.fps();
  }
  auto depth_profiles = depth_sensor.get_stream_profiles();
  VIAM_SDK_LOG(info) << "Found " << depth_profiles.size() << " depth profiles";
  // Log depth profiles
  VIAM_SDK_LOG(info) << "Available depth profiles:";
  for (auto &dp : depth_profiles) {
    auto dsp = dp.as<rs2::video_stream_profile>();
    VIAM_SDK_LOG(info) << "Depth profile: " << dsp.stream_name()
                       << " format: " << dsp.format()
                       << " width: " << dsp.width()
                       << " height: " << dsp.height() << " fps: " << dsp.fps();
  }

  // Find matching profiles (same resolution and FPS)
  for (auto &cp : color_profiles) {
    auto csp = cp.as<rs2::video_stream_profile>();
    if (csp.format() != RS2_FORMAT_RGB8) {
      continue; // Only consider RGB8 format for color
    }
    for (auto &dp : depth_profiles) {
      auto dsp = dp.as<rs2::video_stream_profile>();
      if (dsp.format() != RS2_FORMAT_Z16) {
        continue; // Only consider Z16 format for depth
      }
      // Check if color and depth profiles match in resolution and FPS
      if (checkIfMatchingColorDepthProfiles(csp, dsp)) {
        // Enable matching streams in config
        cfg->enable_stream(RS2_STREAM_COLOR, csp.stream_index(), csp.width(),
                           csp.height(), csp.format(), csp.fps());
        cfg->enable_stream(RS2_STREAM_DEPTH, dsp.stream_index(), dsp.width(),
                           dsp.height(), dsp.format(), dsp.fps());
        return cfg;
      }
    }
  }
  // If no match found, return nullptr
  return nullptr;
}
void registerDevice(std::string serialNumber,
                    std::shared_ptr<rs2::device> dev) {
  VIAM_SDK_LOG(info) << "registering " << serialNumber;
  std::shared_ptr<rs2::pipeline> pipe = std::make_shared<rs2::pipeline>();
  std::shared_ptr<rs2::config> config = createSwD2CAlignConfig(pipe, dev);
  if (config == nullptr) {
    VIAM_SDK_LOG(error)
        << "Current device does not support software depth-to-color "
           "alignment.";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(devices_by_serial_mu());
    std::shared_ptr<ViamRSDevice> my_dev = std::make_shared<ViamRSDevice>();

    my_dev->pipe = pipe;
    my_dev->device = dev;
    my_dev->serial_number = serialNumber;
    my_dev->point_cloud_filter = std::make_shared<PointCloudFilter>();
    my_dev->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    my_dev->config = config;

    devices_by_serial()[serialNumber] = my_dev;
  }

  config->enable_device(serialNumber);
  VIAM_SDK_LOG(info) << "registered " << serialNumber;
}

std::unordered_map<std::string, std::shared_ptr<rs2::device>>
get_removed_devices(rs2::event_information &info) {
  // This function checks if any devices were removed during the callback
  // Create a snapshot of the current devices_by_serial map
  std::vector<std::shared_ptr<rs2::device>> devices_snapshot;
  {
    std::lock_guard<std::mutex> lock(devices_by_serial_mu());
    for (const auto &[serial_number, device] : devices_by_serial()) {
      devices_snapshot.push_back(device->device);
    }
  }

  // Check for removed devices
  std::unordered_map<std::string, std::shared_ptr<rs2::device>> removed_devices;
  for (auto const &device : devices_snapshot) {
    // Check if the device was removed
    if (info.was_removed(*device)) {
      // Add to removed devices list
      removed_devices[device->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)] = device;
    }
  }
  return removed_devices;
}

void deviceChangedCallback(rs2::event_information &info) {
  try {
    // Handling removed devices, if any
    // the callback api does not provide a list of removed devices, so we need
    // to check the current device list for if one or more of them was removed
    auto removed_devices = get_removed_devices(info);
    std::cout << "Removed devices count: " << removed_devices.size()
              << std::endl;
    for (const auto &dev : removed_devices) {
      std::string dev_serial_number =
          dev.second->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout << "Device Removed: " << dev_serial_number << std::endl;
      if (devices_by_serial().count(dev_serial_number) > 0) {
        // Remove from devices_by_serial
        std::cout << "Removing device " << dev_serial_number
                  << " from devices_by_serial" << std::endl;
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        devices_by_serial().erase(dev_serial_number);
      } else
        std::cerr << "Removed device " << dev_serial_number
                  << " not found in devices_by_serial.\n";
    }

    // Handling added devices, if any
    auto added_devices = info.get_new_devices();
    VIAM_SDK_LOG(info) << " Devices added:\n";
    for (const auto &dev : added_devices) {
      std::shared_ptr<rs2::device> device_ptr =
          std::make_shared<rs2::device>(dev);
      printDeviceInfo(*device_ptr);
      registerDevice(device_ptr->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
                     device_ptr);
      {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());

        VIAM_SDK_LOG(info) << "Device added: "
                           << device_ptr->get_info(
                                  RS2_CAMERA_INFO_SERIAL_NUMBER);
        VIAM_SDK_LOG(info) << "serial_by_resource() size: "
                           << serial_by_resource().size();

        for (auto &[resource_name, serial_number] : serial_by_resource()) {
          if (serial_number ==
              device_ptr->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            VIAM_SDK_LOG(info) << "calling startDevice";
            startDevice(serial_number, resource_name);
            serial_by_resource()[resource_name] = serial_number;
          }
        }
      }
    }
  } catch (rs2::error &e) {
    std::cerr << "Error in devices_changed_callback: " << e.what() << std::endl;
  }
}

void startRealsenseSDK(std::shared_ptr<rs2::context> ctx) {
  ctx->set_devices_changed_callback(deviceChangedCallback);
  // This will the initial set of connected devices (i.e. the devices that were
  // connected before the callback was set)
  auto deviceList = ctx->query_devices();
  VIAM_SDK_LOG(info) << "devCount: " << deviceList.size() << "\n";

  for (auto const &dev : deviceList) {
    std::cout << "Device Added: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
              << std::endl;
    auto dev_ptr = std::make_shared<rs2::device>(dev);
    printDeviceInfo(*dev_ptr);
    registerDevice(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), dev_ptr);
  }
  VIAM_SDK_LOG(info) << "Realsense SDK started, devices_by_serial size: "
                     << devices_by_serial().size();
}

} // namespace realsense
