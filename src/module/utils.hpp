#pragma once
#include <boost/range/algorithm/find.hpp>
#include <boost/range/end.hpp>
#include <string>

namespace utils {

template <typename ContainerT>
inline bool contains(std::string const &str, ContainerT const &container) {
  return boost::range::find(container, str) != boost::end(container);
}

// Helper function to create a safe view from RealSense frame data
template <typename FrameT>
auto createFrameDataView(const FrameT &frame, const std::string &frame_type) {
  std::uint8_t *data = static_cast<std::uint8_t *>(frame.get_data());
  uint32_t dataSize = frame.get_data_size();

  if (data == nullptr || dataSize == 0) {
    throw std::runtime_error("[get_point_cloud] " + frame_type +
                             " data is null");
  }

  return boost::make_iterator_range(data, data + dataSize);
}

} // namespace utils
