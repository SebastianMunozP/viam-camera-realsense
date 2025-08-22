#pragma once

#include "time.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <optional>

#include <viam/sdk/log/logging.hpp>

#include <boost/thread/synchronized_value.hpp>
#include <librealsense2/rs.hpp>

namespace realsense {
namespace device {

/********************** UTILITIES ************************/
template<typename DeviceT>
std::optional<std::string> getCameraModel(std::shared_ptr<DeviceT> dev) {
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
            return word;
        }
    }
    return std::nullopt;
}


template <typename DeviceT>
void printDeviceInfo(DeviceT const &dev) {
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

/********************** CALLBACKS ************************/

template <typename FrameT, typename FrameSetT>
void frameCallback(
    FrameT const &frame, std::uint64_t const maxFrameAgeMs,
    boost::synchronized_value<std::shared_ptr<FrameSetT>> &frame_set_) {
  // With callbacks, all synchronized stream will arrive in a single
  // frameset
  auto frameset = std::make_shared<FrameSetT>(frame.template as<FrameSetT>());
  if (not frameset or frameset->size() != 2) {
    std::cerr << "got count other than 2: " << frameset->size() << std::endl;
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

template <typename EventInformationT, typename ViamDeviceT, typename FrameSetT>
void deviceChangedCallback(
    EventInformationT &info,
    std::unordered_set<std::string> const &supported_camera_models,
    boost::synchronized_value<std::shared_ptr<ViamDeviceT>> &device,
    std::string const &required_serial_number,
    boost::synchronized_value<std::shared_ptr<FrameSetT>>
        &frame_set_storage,
    std::uint64_t maxFrameAgeMs) {
  std::cout << "[deviceChangedCallback] Device connection status changed"
            << std::endl;
  try {
    std::shared_ptr<ViamDeviceT> current_device = *device;
    if (current_device and info.was_removed(*current_device->device)) {
      std::cerr << "[deviceChangedCallback] Device removed: "
                << current_device->serial_number << std::endl;
      device = nullptr;
    }

    // Handling added devices, if any
    auto added_devices = info.get_new_devices();
    std::cout << "[deviceChangedCallback] Amount of new devices detected: "
              << added_devices.size() << std::endl;
    std::cout << "[deviceChangedCallback] Required Serial Number: "
              << required_serial_number << std::endl;
    for (const auto &added_device : added_devices) {
      std::string connected_device_serial_number =
          added_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout << "[deviceChangedCallback] New device detected: "
                << connected_device_serial_number << std::endl;

      if (connected_device_serial_number == required_serial_number) {
        std::cerr << "[deviceChangedCallback] New device added: "
                  << connected_device_serial_number << std::endl;
        auto added_device_ptr = std::make_shared<std::decay_t<decltype(added_device)>>(added_device);
        device = createDevice(connected_device_serial_number, added_device_ptr,
                              supported_camera_models);
        startDevice(connected_device_serial_number, device, frame_set_storage,
                    maxFrameAgeMs);
        std::cout << "[deviceChangedCallback] Device Registered: "
                  << required_serial_number << std::endl;
      }
    }
  } catch (rs2::error &e) {
    std::cerr << "[deviceChangedCallback] Error in deviceChangedCallback: "
              << e.what() << std::endl;
  }
}




/************************ STREAM PROFILES ************************/

template <typename VideoStreamProfileT>
bool checkIfMatchingColorDepthProfiles(
    const VideoStreamProfileT &color,
    const VideoStreamProfileT &depth) noexcept {
  if (std::tuple(color.width(), color.height(), color.fps()) ==
      std::tuple(depth.width(), depth.height(), depth.fps())) {
    VIAM_SDK_LOG(info) << "using width: " << color.width()
                       << " height: " << color.height()
                       << " fps: " << color.fps();
    return true;
  }
  return false;
}

// create a config for software depth-to-color alignment
template<typename DeviceT, typename ConfigT, typename ColorSensorT, typename DepthSensorT, typename VideoStreamProfileT>
std::shared_ptr<ConfigT> createSwD2CAlignConfig(std::shared_ptr<DeviceT> dev) {
    auto cfg = std::make_shared<ConfigT>();
    
    // Query all sensors for the device
    auto sensors = dev->query_sensors();

    typename decltype(sensors)::value_type color_sensor;
    typename decltype(sensors)::value_type depth_sensor;
    for (auto &s : sensors) {
        if (s.template is<ColorSensorT>())
            color_sensor = s;
        if (s.template is<DepthSensorT>())
            depth_sensor = s;
    }
    
    // Get stream profiles
    auto color_profiles = color_sensor.get_stream_profiles();
    auto depth_profiles = depth_sensor.get_stream_profiles();
    
    // Find matching profiles
    for (auto &cp : color_profiles) {
        auto csp = cp.template as<VideoStreamProfileT>();
        if (csp.format() != RS2_FORMAT_RGB8) {
            continue;
        }
        for (auto &dp : depth_profiles) {
            auto dsp = dp.template as<VideoStreamProfileT>();
            if (dsp.format() != RS2_FORMAT_Z16) {
                continue;
            }
            if (checkIfMatchingColorDepthProfiles(csp, dsp)) {
                cfg->enable_stream(RS2_STREAM_COLOR, csp.stream_index(), 
                                 csp.width(), csp.height(), csp.format(), csp.fps());
                cfg->enable_stream(RS2_STREAM_DEPTH, dsp.stream_index(), 
                                 dsp.width(), dsp.height(), dsp.format(), dsp.fps());
                return cfg;
            }
        }
    }
    return nullptr;
}

/********************** DEVICE LIFECYCLE ************************/
template <typename ViamDeviceT>
bool destroyDevice(
    boost::synchronized_value<std::shared_ptr<ViamDeviceT>> &dev) noexcept {
  try {
    std::shared_ptr<ViamDeviceT> device = *dev;
    if (!device)
      return false;
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
  } catch (const std::exception &e) {
    VIAM_SDK_LOG(error)
        << "[destroyDevice] Exception caught while destroying device: "
        << e.what();
    return false;
  } catch (...) {
    VIAM_SDK_LOG(error)
        << "[destroyDevice] Unknown exception caught while destroying device";
    return false;
  }
  return true;
}

template <typename ViamDeviceT, typename DeviceT, typename ConfigT, typename ColorSensorT, typename DepthSensorT, typename VideoStreamProfileT>
boost::synchronized_value<std::shared_ptr<ViamDeviceT>>
createDevice(std::string const &serial_number, std::shared_ptr<DeviceT> dev,
             std::unordered_set<std::string> const &supported_camera_models) {
  VIAM_SDK_LOG(info) << "[createDevice] creating device serial number: "
                     << serial_number;
  auto camera_model = getCameraModel(dev);
  if (not camera_model) {
    VIAM_SDK_LOG(error)
        << "[registerDevice] Failed to register camera serial number: "
        << serial_number << " since no camera model found";
    return boost::synchronized_value<std::shared_ptr<ViamDeviceT>>(nullptr);
  }
  VIAM_SDK_LOG(info) << "[registerDevice] Found camera model: "
                     << *camera_model;
  if (supported_camera_models.count(*camera_model) == 0) {
    VIAM_SDK_LOG(error)
        << "[registerDevice] Failed to register camera serial number: "
        << serial_number
        << " since camera model is not D435 or D435i, camera model: "
        << *camera_model;
    return boost::synchronized_value<std::shared_ptr<ViamDeviceT>>(nullptr);
  } else {
    VIAM_SDK_LOG(info) << "[registerDevice] Camera model is supported: "
                       << *camera_model;
  }

  auto config = createSwD2CAlignConfig<DeviceT, ConfigT, ColorSensorT, DepthSensorT, VideoStreamProfileT>(dev);
  if (config == nullptr) {
    VIAM_SDK_LOG(error)
        << "Current device does not support software depth-to-color "
           "alignment.";
    return boost::synchronized_value<std::shared_ptr<ViamDeviceT>>(nullptr);
  }
  std::shared_ptr<ViamDeviceT> my_dev = std::make_shared<ViamDeviceT>();
  my_dev->pipe = std::make_shared<std::decay_t<decltype(*my_dev->pipe)>>();
  my_dev->device = dev;
  my_dev->serial_number = serial_number;
  my_dev->point_cloud_filter = std::make_shared<PointCloudFilter>();
  my_dev->align = std::make_shared<std::decay_t<decltype(*my_dev->align)>>(RS2_STREAM_COLOR);
  my_dev->config = config;

  VIAM_SDK_LOG(info) << "[createDevice] created " << serial_number;
  return my_dev;
}


/********************** STREAMING LIFECYCLE ************************/
template <typename ViamDeviceT, typename FrameSetT>
void startDevice(std::string const &serialNumber,
                 boost::synchronized_value<std::shared_ptr<ViamDeviceT>> dev,
                 boost::synchronized_value<std::shared_ptr<FrameSetT>>
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
                                             auto const &frame) {
    frameCallback(frame, maxFrameAgeMs, frame_set_storage);
  });
  dev_ptr->started = true;
  VIAM_SDK_LOG(info) << "[startDevice]  device started " << serialNumber;
}

template <typename ViamDeviceT>
bool stopDevice(
    boost::synchronized_value<std::shared_ptr<ViamDeviceT>> &dev) noexcept {
  try {
    std::shared_ptr<ViamDeviceT> dev_ptr = *dev;
    if (not dev_ptr) {
      VIAM_SDK_LOG(error)
          << "[stopDevice] trying to stop a device that does not exist";
      return false;
    }
    if (not dev_ptr->started) {
      VIAM_SDK_LOG(error)
          << "[stopDevice] unable to stop device that is not currently running "
          << dev_ptr->serial_number;
      return false;
    }

    dev_ptr->pipe->stop();
    dev_ptr->started = false;
    return true;
  } catch (const std::exception &e) {
    VIAM_SDK_LOG(error)
        << "[stopDevice] Exception caught while stopping device: " << e.what();
  } catch (...) {
    VIAM_SDK_LOG(error)
        << "[stopDevice] Unknown exception caught while stopping device";
  }
  return false;
}
}
}