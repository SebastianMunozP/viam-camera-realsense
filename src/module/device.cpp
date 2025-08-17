#include "device.hpp"
#include "time.hpp"

#include <iostream>
#include <optional>
#include <unordered_set>

#include <viam/sdk/log/logging.hpp>

#include <boost/thread/synchronized_value.hpp>

namespace realsense {
namespace device {

/***************************** UTILS *************************************/

void printDeviceInfo(rs2::device const &dev) noexcept {
  std::stringstream info;
  if (dev.supports(RS2_CAMERA_INFO_NAME)) {
    info << "DeviceInfo:\n"
         << "  Name:                           "
         << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
    info << "  Serial Number:                  "
         << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_PRODUCT_LINE)) {
    info << "  Product Line:                   "
         << dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_PRODUCT_ID)) {
    info << "  Product ID:                      "
         << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
    info << "  USB Type Descriptor:            "
         << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION)) {
    info << "  Firmware Version:               "
         << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
    info << "  Recommended Firmware Version:   "
         << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)
         << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
    info << "  Firmware Update ID:   "
         << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)) {
    info << "  Physical Port:   " << dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT)
         << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_DEBUG_OP_CODE)) {
    info << "  Debug OP Code:   " << dev.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE)
         << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_ADVANCED_MODE)) {
    info << "  Advanced Mode:   " << dev.get_info(RS2_CAMERA_INFO_ADVANCED_MODE)
         << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_PRODUCT_ID)) {
    info << "  Product ID:   " << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID)
         << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED)) {
    info << "  Camera Locked:   " << dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED)
         << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)) {
    info << "  ASIC Serial Number:             "
         << dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
    info << "  USB Type Descriptor:            "
         << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_DFU_DEVICE_PATH)) {
    info << "  DFU Device Path:            "
         << dev.get_info(RS2_CAMERA_INFO_DFU_DEVICE_PATH) << std::endl;
  }
  if (dev.supports(RS2_CAMERA_INFO_IP_ADDRESS)) {
    info << "  IP Address:            "
         << dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS) << std::endl;
  }
  VIAM_SDK_LOG(info) << info.str();
}

std::optional<std::string>
getCameraModel(std::shared_ptr<rs2::device> dev) noexcept {
  if (not dev->supports(RS2_CAMERA_INFO_NAME)) {
    return std::nullopt;
  }
  std::string camera_info = dev->get_info(RS2_CAMERA_INFO_NAME);
  // Model is the 3rd word in this string
  std::istringstream iss(camera_info);
  std::string word;
  int word_count = 0;
  while (iss >> word) {
    word_count++;
    if (word_count == 3) {
      std::string camera_model = word;
      return camera_model;
    }
  }
  return std::nullopt;
}

/***************************** STREAM PROFILES
 * *************************************/

bool checkIfMatchingColorDepthProfiles(
    const rs2::video_stream_profile &color,
    const rs2::video_stream_profile &depth) noexcept {
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
  auto depth_profiles = depth_sensor.get_stream_profiles();
  VIAM_SDK_LOG(info) << "Found " << depth_profiles.size() << " depth profiles";

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

/***************************** CALLBACKS *************************************/

void frameCallback(
    rs2::frame const &frame, std::uint64_t const maxFrameAgeMs,
    boost::synchronized_value<std::shared_ptr<rs2::frameset>> &frame_set_) {
  // With callbacks, all synchronized stream will arrive in a single
  // frameset
  auto frameset = std::make_shared<rs2::frameset>(frame.as<rs2::frameset>());
  if (not frameset or frameset->size() != 2) {
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

  double nowMs = time::getNowMs();
  double colorAge = nowMs - color_frame.get_timestamp();
  double depthAge = nowMs - depth_frame.get_timestamp();

  if (colorAge > maxFrameAgeMs) {
    std::cerr << "[frame_callback] received color frame is too stale, age: "
              << colorAge << "ms" << std::endl;
  }

  if (depthAge > maxFrameAgeMs) {
    std::cerr << "[frame_callback] received depth frame is too stale, age: "
              << depthAge << "ms" << std::endl;
  }

  frame_set_ = frameset;
}

void deviceChangedCallback(
    rs2::event_information &info,
    std::unordered_set<std::string> const &supported_camera_models,
    boost::synchronized_value<std::shared_ptr<ViamRSDevice>> &device,
    std::string const &required_serial_number,
    boost::synchronized_value<std::shared_ptr<rs2::frameset>>
        &frame_set_storage,
    std::uint64_t maxFrameAgeMs) {
  std::cout << "[deviceChangedCallback] Device connection status changed"
            << std::endl;
  try {
    std::shared_ptr<ViamRSDevice> current_device = *device;
    if (info.was_removed(*current_device->device)) {
      std::cerr << "[deviceChangedCallback] Device removed: "
                << current_device->serial_number << std::endl;
      device = nullptr;
    }

    // Handling added devices, if any
    auto added_devices = info.get_new_devices();
    std::cout << "[deviceChangedCallback] Amount of devices added: "
              << added_devices.size() << std::endl;
    for (const auto &added_device : added_devices) {
      std::string connected_device_serial_number =
          added_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

      if (connected_device_serial_number == required_serial_number) {
        std::cout << "[deviceChangedCallback] New device added: "
                  << connected_device_serial_number << std::endl;
        auto added_device_ptr = std::make_shared<rs2::device>(added_device);
        device = createDevice(connected_device_serial_number, added_device_ptr,
                              supported_camera_models);
        std::shared_ptr<ViamRSDevice> current_device = *device;
        startDevice(connected_device_serial_number, current_device,
                    frame_set_storage, maxFrameAgeMs);
        std::cout << "[deviceChangedCallback] Device Registered: "
                  << required_serial_number << std::endl;
      }
    }
  } catch (rs2::error &e) {
    std::cerr << "[deviceChangedCallback] Error in deviceChangedCallback: "
              << e.what() << std::endl;
  }
}

/***************************** DEVICE CONTROL
 * *************************************/

void stopDevice(boost::synchronized_value<std::shared_ptr<ViamRSDevice>> &dev) {
  std::shared_ptr<ViamRSDevice> dev_ptr = *dev;
  if (not dev_ptr) {
    VIAM_SDK_LOG(error)
        << "[stopDevice] trying to stop a device that does not exist";
    return;
  }
  if (not dev_ptr->started) {
    VIAM_SDK_LOG(error)
        << "[stopDevice] unable to stop device that is not currently running "
        << dev_ptr->serial_number;
    return;
  }

  dev_ptr->pipe->stop();
  dev_ptr->started = false;
}

void startDevice(std::string serialNumber,
                 boost::synchronized_value<std::shared_ptr<ViamRSDevice>> dev,
                 boost::synchronized_value<std::shared_ptr<rs2::frameset>>
                     &frame_set_storage,
                 std::uint64_t const maxFrameAgeMs) {
  VIAM_SDK_LOG(info) << "[startDevice] starting device " << serialNumber;
  std::shared_ptr<ViamRSDevice> dev_ptr = *dev;
  if (dev_ptr->started) {
    std::ostringstream buffer;
    buffer << "[startDevice] unable to start already started device "
           << serialNumber;
    throw std::invalid_argument(buffer.str());
  }

  dev_ptr->config->enable_device(serialNumber);
  dev_ptr->pipe->start(*dev_ptr->config, [maxFrameAgeMs, &frame_set_storage](
                                             rs2::frame const &frame) {
    frameCallback(frame, maxFrameAgeMs, frame_set_storage);
  });
  dev_ptr->started = true;
  VIAM_SDK_LOG(info) << "[startDevice]  device started " << serialNumber;
}

std::shared_ptr<ViamRSDevice>
createDevice(std::string serial_number, std::shared_ptr<rs2::device> dev,
             std::unordered_set<std::string> const &supported_camera_models) {
  VIAM_SDK_LOG(info) << "[createDevice] creating device serial number: "
                     << serial_number;
  auto camera_model = getCameraModel(dev);
  if (not camera_model) {
    VIAM_SDK_LOG(error)
        << "[registerDevice] Failed to register camera serial number: "
        << serial_number << " since no camera model found";
    return nullptr;
  }
  VIAM_SDK_LOG(info) << "[registerDevice] Found camera model: "
                     << *camera_model;
  if (supported_camera_models.count(*camera_model) == 0) {
    VIAM_SDK_LOG(error)
        << "[registerDevice] Failed to register camera serial number: "
        << serial_number
        << " since camera model is not D435 or D435i, camera model: "
        << *camera_model;
    return nullptr;
  } else {
    VIAM_SDK_LOG(info) << "[registerDevice] Camera model is supported: "
                       << *camera_model;
  }

  std::shared_ptr<rs2::pipeline> pipe = std::make_shared<rs2::pipeline>();
  std::shared_ptr<rs2::config> config = createSwD2CAlignConfig(pipe, dev);
  if (config == nullptr) {
    VIAM_SDK_LOG(error)
        << "Current device does not support software depth-to-color "
           "alignment.";
    return nullptr;
  }

  std::shared_ptr<ViamRSDevice> my_dev = std::make_shared<ViamRSDevice>();

  my_dev->pipe = pipe;
  my_dev->device = dev;
  my_dev->serial_number = serial_number;
  my_dev->point_cloud_filter = std::make_shared<PointCloudFilter>();
  my_dev->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
  my_dev->config = config;

  VIAM_SDK_LOG(info) << "[createDevice] created " << serial_number;
  return my_dev;
}

void destroyDevice(
    boost::synchronized_value<std::shared_ptr<ViamRSDevice>> &dev) {
  std::shared_ptr<ViamRSDevice> device = *dev;
  if (!device)
    return;
  VIAM_SDK_LOG(info) << "[destroyDevice] destroying device "
                     << device->serial_number;

  // Stop streaming if still running
  if (device->started) {
    VIAM_SDK_LOG(info) << "[destroyDevice] stopping pipe "
                       << device->serial_number;
    device->pipe->stop();
    device->started = false;
  }

  // Clear all resources
  VIAM_SDK_LOG(info) << "[destroyDevice] clearing resources "
                     << device->serial_number;
  device->pipe.reset();
  device->device.reset();
  device->config.reset();
  device->align.reset();
  device->point_cloud_filter.reset();

  VIAM_SDK_LOG(info) << "[destroyDevice] device destroyed: "
                     << device->serial_number;
  dev = nullptr;
}

} // namespace device
} // namespace realsense
