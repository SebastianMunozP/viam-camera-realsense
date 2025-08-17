#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

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
  size_t frame_count = 0;
  std::chrono::steady_clock::time_point last_report =
      std::chrono::steady_clock::now();
};

// Utilities
void printDeviceInfo(const rs2::device &dev) noexcept;
void deviceChangedCallback(
    rs2::event_information &info,
    std::unordered_set<std::string> const &supported_camera_models,
    boost::synchronized_value<std::shared_ptr<ViamRSDevice>> &device,
    std::string const &required_serial_number,
    boost::synchronized_value<std::shared_ptr<rs2::frameset>>
        &frame_set_storage,
    std::uint64_t maxFrameAgeMs);

// Streaming lifecycle
void startDevice(
    std::string serialNumber,
    boost::synchronized_value<std::shared_ptr<ViamRSDevice>> dev_ptr,
    boost::synchronized_value<std::shared_ptr<rs2::frameset>>
        &frame_set_storage,
    std::uint64_t maxFrameAgeMs);
void stopDevice(boost::synchronized_value<std::shared_ptr<ViamRSDevice>> &dev);

// Device lifecycle
std::shared_ptr<ViamRSDevice>
createDevice(std::string serial_number, std::shared_ptr<rs2::device> dev,
             std::unordered_set<std::string> const &supported_camera_models);
void destroyDevice(
    boost::synchronized_value<std::shared_ptr<ViamRSDevice>> &dev);

} // namespace device
} // namespace realsense