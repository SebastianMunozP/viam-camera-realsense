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
  std::string serial_number;
  std::shared_ptr<rs2::device> device;
  bool started;
  std::shared_ptr<rs2::pipeline> pipe;
  std::shared_ptr<PointCloudFilter> point_cloud_filter;
  std::shared_ptr<rs2::align> align;
  std::shared_ptr<rs2::config> config;
};
/********************** UTILITIES ************************/
template <typename DeviceT>
void printDeviceInfo(DeviceT const &dev);


/********************** CALLBACKS ************************/
template <typename EventInformationT, typename ViamDeviceT, typename FrameSetT>
void deviceChangedCallback(
    EventInformationT &info,
    std::unordered_set<std::string> const &supported_camera_models,
    boost::synchronized_value<std::shared_ptr<ViamDeviceT>> &device,
    std::string const &required_serial_number,
    boost::synchronized_value<std::shared_ptr<FrameSetT>>
        &frame_set_storage,
    std::uint64_t maxFrameAgeMs);

template <typename FrameT, typename FrameSetT>
void frameCallback(
    FrameT const &frame, std::uint64_t const maxFrameAgeMs,
    boost::synchronized_value<std::shared_ptr<FrameSetT>> &frame_set_);


/********************** DEVICE LIFECYCLE ************************/
template <typename ViamDeviceT = ViamRSDevice, typename DeviceT = rs2::device, typename ConfigT = rs2::config, typename ColorSensorT = rs2::color_sensor, typename DepthSensorT = rs2::depth_sensor, typename VideoStreamProfileT = rs2::video_stream_profile>
boost::synchronized_value<std::shared_ptr<ViamDeviceT>>
createDevice(std::string const &serial_number, std::shared_ptr<DeviceT> dev,
             std::unordered_set<std::string> const &supported_camera_models);

template <typename ViamDeviceT>
bool destroyDevice(
    boost::synchronized_value<std::shared_ptr<ViamDeviceT>> &dev) noexcept;


/********************** STREAMING LIFECYCLE ************************/
template <typename ViamDeviceT, typename FrameSetT>
void startDevice(std::string const &serialNumber,
                 boost::synchronized_value<std::shared_ptr<ViamDeviceT>> dev,
                 boost::synchronized_value<std::shared_ptr<FrameSetT>>
                     &frame_set_storage,
                 std::uint64_t const maxFrameAgeMs);

template <typename ViamDeviceT>
bool stopDevice(
    boost::synchronized_value<std::shared_ptr<ViamDeviceT>> &dev) noexcept;

} // namespace device
} // namespace realsense

#include "device_impl.hpp"