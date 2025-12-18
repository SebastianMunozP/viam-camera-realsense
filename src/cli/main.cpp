// This is a command line utility to interact with the
// orbbec SDK.
// It is only intended as a viam developer debugging tool

#include <math.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <vector>

void printDeviceInfo(rs2::device const &dev) {
  try {
    std::stringstream info;
    if (dev.supports(RS2_CAMERA_INFO_NAME)) {
      std::cout << "DeviceInfo:\n"
           << "  Name:                           "
           << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
      std::cout << "  Serial Number:                  "
           << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PRODUCT_LINE)) {
      std::cout << "  Product Line:                   "
           << dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PRODUCT_ID)) {
      std::cout << "  Product ID:                      "
           << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
      std::cout << "  USB Type Descriptor:            "
           << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION)) {
      std::cout << "  Firmware Version:               "
           << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
      std::cout << "  Recommended Firmware Version:   "
           << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)
           << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
      std::cout << "  Firmware Update ID:   "
           << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)) {
      std::cout << "  Physical Port:   "
           << dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_DEBUG_OP_CODE)) {
      std::cout << "  Debug OP Code:   "
           << dev.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_ADVANCED_MODE)) {
      std::cout << "  Advanced Mode:   "
           << dev.get_info(RS2_CAMERA_INFO_ADVANCED_MODE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED)) {
      std::cout << "  Camera Locked:   "
           << dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)) {
      std::cout << "  ASIC Serial Number:             "
           << dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_DFU_DEVICE_PATH)) {
      std::cout << "  DFU Device Path:            "
           << dev.get_info(RS2_CAMERA_INFO_DFU_DEVICE_PATH) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_IP_ADDRESS)) {
      std::cout << "  IP Address:            "
           << dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS) << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "[printDeviceInfo] Failed to retrieve device info with error: "
              << e.what()
              << ". Device may be in an invalid state or have firmware compatibility "
                 "issues." << std::endl;
  }
}


int main() {
    std::cout << "starting realsense program" << std::endl;

    rs2::context ctx;
    // rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    auto const device_list = ctx.query_devices();
    std::cout << "Number of connected devices: " << device_list.size() << "\n";
    for (size_t i = 0; i < device_list.size(); i++) {
        try {
            rs2::device dev = device_list[i];
            printDeviceInfo(dev);
        } catch (const std::exception &e) {
            std::cerr << "[main] Failed to process device with error: " << e.what() << std::endl;
        }
    }

    std::cout << "waiting for key press\n";
    std::cin.get();
    std::cout << "stopping realsense program" << std::endl;
    return 0;
}
