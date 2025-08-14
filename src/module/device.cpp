#include "device.hpp"
#include "time.hpp"

#include <iostream>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include <viam/sdk/log/logging.hpp>
namespace realsense {
namespace device {

// REALSENSE SDK DEVICE REGISTRY START
void printDeviceInfo(rs2::device const &dev) {
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
void printDeviceList(const std::shared_ptr<rs2::device_list> devList) {
  std::for_each(devList->begin(), devList->end(),
                [](const rs2::device &dev) { printDeviceInfo(dev); });
}

void startDevice(std::string serialNumber,
                 std::shared_ptr<ViamRSDevice> dev_ptr,
                 std::mutex &frame_set_by_serial_mu,
                 std::unordered_map<std::string, std::shared_ptr<rs2::frameset>>
                     &frame_set_by_serial,
                 std::uint64_t maxFrameAgeMs) {
  VIAM_SDK_LOG(info) << "[startDevice] starting device " << serialNumber;
  if (dev_ptr->started) {
    std::ostringstream buffer;
    buffer << "[startDevice] unable to start already started device "
           << serialNumber;
    throw std::invalid_argument(buffer.str());
  }

  auto frameCallback = [serialNumber, dev_ptr, maxFrameAgeMs,
                        &frame_set_by_serial_mu,
                        &frame_set_by_serial](rs2::frame const &frame) {
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

      std::lock_guard<std::mutex> lock(frame_set_by_serial_mu);
      double nowMs = time::getNowMs();
      time::logIfTooOld(std::cerr, nowMs, color_frame.get_timestamp(),
                        maxFrameAgeMs,
                        "[frame_callback] received color frame is too stale");
      time::logIfTooOld(std::cerr, nowMs, depth_frame.get_timestamp(),
                        maxFrameAgeMs,
                        "[frame_callback] received depth frame is too stale");

      auto it = frame_set_by_serial.find(serialNumber);
      if (it != frame_set_by_serial.end()) {
        rs2::frame prevColor = it->second->get_color_frame();
        rs2::frame prevDepth = it->second->get_depth_frame();
        if (prevColor and prevDepth) {
          time::logIfTooOld(
              std::cerr, color_frame.get_timestamp(), prevColor.get_timestamp(),
              maxFrameAgeMs,
              "[frameCallback] previous color frame is too stale");
          time::logIfTooOld(
              std::cerr, depth_frame.get_timestamp(), prevDepth.get_timestamp(),
              maxFrameAgeMs,
              "[frameCallback] previous depth frame is too stale");
        }
      }
      frame_set_by_serial[serialNumber] =
          std::make_shared<rs2::frameset>(frame.as<rs2::frameset>());
    } else {
      // Stream that bypass synchronization (such as IMU) will produce single
      // frames
      std::cerr << "got non 2 a frameset: " << frame.get_profile().stream_name()
                << std::endl;
      return;
    }
  };

  dev_ptr->pipe->start(*dev_ptr->config, std::move(frameCallback));
  dev_ptr->started = true;
  VIAM_SDK_LOG(info) << "[startDevice]  device started " << serialNumber;
}

void stopDevice(
    std::string serialNumber, std::string resourceName,
    std::shared_ptr<ViamRSDevice> dev_ptr, std::mutex &serial_by_resource_mu,
    std::unordered_map<std::string, std::string> &serial_by_resource) {
  if (not dev_ptr or not dev_ptr->started) {
    VIAM_SDK_LOG(error)
        << "[stopDevice] unable to stop device that is not currently running "
        << serialNumber;
    return;
  }

  dev_ptr->pipe->stop();
  dev_ptr->started = false;
  {
    std::lock_guard<std::mutex> lock(serial_by_resource_mu);
    serial_by_resource.erase(resourceName);
  }
}

bool isDeviceConnected(const std::string &serial_number) {
  auto ctx = std::make_shared<rs2::context>();
  auto devices = ctx->query_devices();
  for (const auto &dev : devices) {
    if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial_number) {
      return true;
    }
  }
  return false;
}

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

std::optional<std::string> getCameraModel(std::shared_ptr<rs2::device> dev) {
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

void registerDevice(
    std::string serialNumber, std::shared_ptr<rs2::device> dev,
    std::unordered_set<std::string> const &supported_camera_models,
    std::mutex &devices_by_serial_mu,
    std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>>
        &devices_by_serial) {
  VIAM_SDK_LOG(info) << "[registerDevice] registering " << serialNumber;
  auto camera_model = getCameraModel(dev);
  if (not camera_model) {
    VIAM_SDK_LOG(error)
        << "[registerDevice] Failed to register camera serial number: "
        << serialNumber << " since no camera model found";
    return;
  }
  VIAM_SDK_LOG(info) << "[registerDevice] Found camera model: "
                     << *camera_model;
  if (supported_camera_models.count(*camera_model) == 0) {
    VIAM_SDK_LOG(error)
        << "[registerDevice] Failed to register camera serial number: "
        << serialNumber
        << " since camera model is not D435 or D435i, camera model: "
        << *camera_model;
    return;
  }

  std::shared_ptr<rs2::pipeline> pipe = std::make_shared<rs2::pipeline>();
  std::shared_ptr<rs2::config> config = createSwD2CAlignConfig(pipe, dev);
  if (config == nullptr) {
    VIAM_SDK_LOG(error)
        << "Current device does not support software depth-to-color "
           "alignment.";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(devices_by_serial_mu);
    std::shared_ptr<ViamRSDevice> my_dev = std::make_shared<ViamRSDevice>();

    my_dev->pipe = pipe;
    my_dev->device = dev;
    my_dev->serial_number = serialNumber;
    my_dev->point_cloud_filter = std::make_shared<PointCloudFilter>();
    my_dev->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    my_dev->config = config;

    devices_by_serial[serialNumber] = my_dev;
  }

  config->enable_device(serialNumber);
  VIAM_SDK_LOG(info) << "registered " << serialNumber;
}

std::unordered_map<std::string, std::shared_ptr<rs2::device>>
get_removed_devices(
    rs2::event_information &info, std::mutex &devices_by_serial_mu,
    std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>>
        &devices_by_serial) {
  // This function checks if any devices were removed during the callback
  // Create a snapshot of the current devices_by_serial map
  std::vector<std::shared_ptr<rs2::device>> devices_snapshot;
  {
    std::lock_guard<std::mutex> lock(devices_by_serial_mu);
    for (const auto &[serial_number, device] : devices_by_serial) {
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

void deviceChangedCallback(
    rs2::event_information &info,
    std::unordered_set<std::string> const &supported_camera_models,
    std::mutex &devices_by_serial_mu,
    std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>>
        &devices_by_serial,
    std::mutex &serial_by_resource_mu,
    std::unordered_map<std::string, std::string> &serial_by_resource,
    std::mutex &frame_set_by_serial_mu,
    std::unordered_map<std::string, std::shared_ptr<rs2::frameset>>
        &frame_set_by_serial,
    std::uint64_t maxFrameAgeMs) {
  try {
    // Handling removed devices, if any
    // the callback api does not provide a list of removed devices, so we need
    // to check the current device list for if one or more of them was removed
    auto removed_devices =
        get_removed_devices(info, devices_by_serial_mu, devices_by_serial);
    std::cout << "Removed devices count: " << removed_devices.size()
              << std::endl;
    for (const auto &dev : removed_devices) {
      std::string dev_serial_number =
          dev.second->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout << "Device Removed: " << dev_serial_number << std::endl;
      if (devices_by_serial.count(dev_serial_number) > 0) {
        // Remove from devices_by_serial
        std::cout << "Removing device " << dev_serial_number
                  << " from devices_by_serial" << std::endl;
        std::lock_guard<std::mutex> lock(devices_by_serial_mu);
        devices_by_serial.erase(dev_serial_number);
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
                     device_ptr, supported_camera_models, devices_by_serial_mu,
                     devices_by_serial);
      {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu);

        VIAM_SDK_LOG(info) << "Device added: "
                           << device_ptr->get_info(
                                  RS2_CAMERA_INFO_SERIAL_NUMBER);
        VIAM_SDK_LOG(info) << "serial_by_resource() size: "
                           << serial_by_resource.size();

        for (auto &[resource_name, serial_number] : serial_by_resource) {
          if (serial_number ==
              device_ptr->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            VIAM_SDK_LOG(info) << "calling startDevice";
            startDevice(serial_number, devices_by_serial[serial_number],
                        frame_set_by_serial_mu, frame_set_by_serial,
                        maxFrameAgeMs);
            serial_by_resource[resource_name] = serial_number;
          }
        }
      }
    }
  } catch (rs2::error &e) {
    std::cerr << "Error in devices_changed_callback: " << e.what() << std::endl;
  }
}

} // namespace device
} // namespace realsense
