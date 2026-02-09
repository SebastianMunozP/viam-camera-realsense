#include <librealsense2/rs.hpp>

#pragma once

namespace realsense {
namespace sensors {

enum class SensorType { depth, color, unknown };

std::string sensor_type_to_string(SensorType const sensor_type) {
  switch (sensor_type) {
  case SensorType::depth:
    return "depth";
  case SensorType::color:
    return "color";
  default:
    return "unknown";
  }
}

SensorType string_to_sensor_type(std::string const &sensor_type) {
  if (sensor_type == "depth") {
    return SensorType::depth;
  } else if (sensor_type == "color") {
    return SensorType::color;
  }
  return SensorType::unknown;
}

template <typename SensorT> SensorType get_sensor_type(SensorT const &sensor) {
  if (sensor.template is<rs2::depth_sensor>()) {
    return SensorType::depth;
  } else if (sensor.template is<rs2::color_sensor>()) {
    return SensorType::color;
  }
  return SensorType::unknown;
}
} // namespace sensors
} // namespace realsense