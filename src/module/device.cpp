#include "device.hpp"
#include "time.hpp"

#include <iostream>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include <viam/sdk/log/logging.hpp>

#include <boost/thread/synchronized_value.hpp>

namespace realsense {
namespace device {

std::shared_ptr<rs2::frameset> getFramesetBySerial(
    const std::string &serial_number,
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<rs2::frameset>>>
        &frame_set_by_serial) noexcept {
  auto it = frame_set_by_serial->find(serial_number);
  if (it == frame_set_by_serial->end()) {
    return nullptr;
  }
  return it->second;
}

std::shared_ptr<device::ViamRSDevice> getDeviceBySerial(
    std::string const &serial_number,
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<device::ViamRSDevice>>>
        &devices_by_serial) noexcept {
  auto it = devices_by_serial->find(serial_number);
  if (it == devices_by_serial->end()) {
    return nullptr;
  }
  return it->second;
}


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
void printDeviceList(const std::shared_ptr<rs2::device_list> devList) {
  std::for_each(devList->begin(), devList->end(),
                [](const rs2::device &dev) { printDeviceInfo(dev); });
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

std::unordered_map<std::string, std::shared_ptr<rs2::device>>
get_removed_devices(
    rs2::event_information &info,
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>>>
        &devices_by_serial) {
  // This function checks if any devices were removed during the callback
  // Create a snapshot of the current devices_by_serial map
  std::vector<std::shared_ptr<rs2::device>> devices_snapshot;
  {
    auto &&devices_by_serial_synch = devices_by_serial.synchronize();
    for (const auto &[serial_number, device] : *devices_by_serial_synch) {
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
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>>>
        &devices_by_serial,
    boost::synchronized_value<std::unordered_map<std::string, std::string>>
        &serial_by_resource,
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<rs2::frameset>>>
        &frame_set_by_serial,
    std::uint64_t maxFrameAgeMs) {
  try {
    // Handling removed devices, if any
    // the callback api does not provide a list of removed devices, so we need
    // to check the current device list for if one or more of them was removed
    auto removed_devices = get_removed_devices(info, devices_by_serial);
    VIAM_SDK_LOG(info) << "[deviceChangedCallback] Removed devices count: "
                       << removed_devices.size();
    for (const auto &dev : removed_devices) {
      std::string dev_serial_number =
          dev.second->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      VIAM_SDK_LOG(info) << "[deviceChangedCallback] Device Removed: "
                         << dev_serial_number;
      if (devices_by_serial->count(dev_serial_number) > 0) {
        // Remove from devices_by_serial
        VIAM_SDK_LOG(info) << "[deviceChangedCallback] Removing device "
                           << dev_serial_number << " from devices_by_serial";
        devices_by_serial->erase(dev_serial_number);
      } else
        VIAM_SDK_LOG(error)
            << "[deviceChangedCallback] Removed device " << dev_serial_number
            << " not found in devices_by_serial.";
    }

    // Handling added devices, if any
    auto added_devices = info.get_new_devices();
    VIAM_SDK_LOG(info) << " Devices added:\n";
    for (const auto &dev : added_devices) {
      std::shared_ptr<rs2::device> device_ptr =
          std::make_shared<rs2::device>(dev);
      printDeviceInfo(*device_ptr);
      registerDevice(device_ptr->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
                     device_ptr, supported_camera_models, devices_by_serial);
      {
        auto &&serial_by_resource_synch = serial_by_resource.synchronize();
        VIAM_SDK_LOG(info) << "[deviceChangedCallback] Device registered: "
                           << device_ptr->get_info(
                                  RS2_CAMERA_INFO_SERIAL_NUMBER);
        VIAM_SDK_LOG(info) << "[deviceChangedCallback] devices_by_serial size: "
                           << devices_by_serial->size();
        VIAM_SDK_LOG(info)
            << "[deviceChangedCallback] serial_by_resource size: "
            << serial_by_resource_synch->size();

        for (auto &[resource_name, serial_number] : *serial_by_resource_synch) {
          if (serial_number ==
              device_ptr->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            VIAM_SDK_LOG(info) << "[deviceChangedCallback] calling startDevice";
            auto device_ptr = devices_by_serial->at(serial_number);
            startDevice(serial_number, device_ptr, frame_set_by_serial,
                        maxFrameAgeMs);
          }
        }
      }
    }
  } catch (rs2::error &e) {
    VIAM_SDK_LOG(error)
        << "[deviceChangedCallback] Error in devices_changed_callback: "
        << e.what() << std::endl;
  }
}


void frameCallback(
    rs2::frame const &frame, std::string const &serialNumber,
    std::shared_ptr<ViamRSDevice> dev_ptr, std::uint64_t const maxFrameAgeMs,
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<rs2::frameset>>>
        &frame_set_by_serial) {
  // With callbacks, all synchronized stream will arrive in a single
  // frameset
  auto frameset = std::make_shared<rs2::frameset>(frame.as<rs2::frameset>());
  if (not frameset or frameset->size() != 2) {
    VIAM_SDK_LOG(error) << "got non 2 frame count: " << frameset->size()
                        << std::endl;
    return;
  }
  auto color_frame = frameset->get_color_frame();
  if (not color_frame) {
    VIAM_SDK_LOG(error) << "no color frame" << std::endl;
    return;
  }

  auto depth_frame = frameset->get_depth_frame();
  if (not depth_frame) {
    VIAM_SDK_LOG(error) << "no depth frame" << std::endl;
    return;
  }

  double nowMs = time::getNowMs();
  double colorAge = nowMs - color_frame.get_timestamp();
  double depthAge = nowMs - depth_frame.get_timestamp();

  if (colorAge > maxFrameAgeMs) {
    VIAM_SDK_LOG(error)
        << "[frame_callback] received color frame is too stale, age: "
        << colorAge << "ms";
  }

  if (depthAge > maxFrameAgeMs) {
    VIAM_SDK_LOG(error)
        << "[frame_callback] received depth frame is too stale, age: "
        << depthAge << "ms";
  }

  auto &&frame_set_by_serial_synch = frame_set_by_serial.synchronize();
  auto it = frame_set_by_serial_synch->find(serialNumber);
  if (it != frame_set_by_serial_synch->end()) {
    rs2::frame prevColor = it->second->get_color_frame();
    rs2::frame prevDepth = it->second->get_depth_frame();
    if (prevColor and prevDepth) {
      time::logIfTooOld(std::cerr, color_frame.get_timestamp(),
                        prevColor.get_timestamp(), maxFrameAgeMs,
                        "[frameCallback] previous color frame is too stale");
      time::logIfTooOld(std::cerr, depth_frame.get_timestamp(),
                        prevDepth.get_timestamp(), maxFrameAgeMs,
                        "[frameCallback] previous depth frame is too stale");
    }
  }
  frame_set_by_serial_synch->insert_or_assign(serialNumber, frameset);
}

void startDevice(
    std::string serialNumber, std::shared_ptr<ViamRSDevice> dev_ptr,
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<rs2::frameset>>>
        &frame_set_by_serial,
    std::uint64_t maxFrameAgeMs) {
  VIAM_SDK_LOG(info) << "[startDevice] starting device " << serialNumber;
  if (dev_ptr->started) {
    std::ostringstream buffer;
    buffer << "[startDevice] unable to start already started device "
           << serialNumber;
    throw std::invalid_argument(buffer.str());
  }

  dev_ptr->pipe->start(*dev_ptr->config,
                       [serialNumber, dev_ptr, maxFrameAgeMs,
                        &frame_set_by_serial](rs2::frame const &frame) {
                         frameCallback(frame, serialNumber, dev_ptr,
                                       maxFrameAgeMs, frame_set_by_serial);
                       });
  dev_ptr->started = true;
  VIAM_SDK_LOG(info) << "[startDevice]  device started " << serialNumber;
}

void stopDevice(
    std::string serialNumber, std::string resourceName,
    std::shared_ptr<ViamRSDevice> dev_ptr,
    boost::synchronized_value<std::unordered_map<std::string, std::string>>
        &serial_by_resource) {
  if (not dev_ptr or not dev_ptr->started) {
    VIAM_SDK_LOG(error)
        << "[stopDevice] unable to stop device that is not currently running "
        << serialNumber;
    return;
  }

  dev_ptr->pipe->stop();
  dev_ptr->started = false;
  serial_by_resource->erase(resourceName);
}

void registerDevice(
    std::string serialNumber, std::shared_ptr<rs2::device> dev,
    std::unordered_set<std::string> const &supported_camera_models,
    boost::synchronized_value<
        std::unordered_map<std::string, std::shared_ptr<ViamRSDevice>>>
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
    return;
  }

  std::shared_ptr<ViamRSDevice> my_dev = std::make_shared<ViamRSDevice>();

  my_dev->pipe = pipe;
  my_dev->device = dev;
  my_dev->serial_number = serialNumber;
  my_dev->point_cloud_filter = std::make_shared<PointCloudFilter>();
  my_dev->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
  my_dev->config = config;

  devices_by_serial->insert_or_assign(serialNumber, my_dev);

  config->enable_device(serialNumber);
  VIAM_SDK_LOG(info) << "[registerDevice] registered " << serialNumber;
}

} // namespace device
} // namespace realsense
