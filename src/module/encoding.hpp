
#pragma once
#include <viam/sdk/components/camera.hpp>

#include <librealsense2/rs.hpp>
#include <turbojpeg.h>

#include <cstdint>
#include <vector>
namespace realsense {
namespace encoding {

viam::sdk::Camera::raw_image encodeDepthRAWToResponse(const std::uint8_t *data,
                                                      const uint width,
                                                      const uint height);

viam::sdk::Camera::raw_image encodeJPEGToResponse(const std::uint8_t *data,
                                                  const uint width,
                                                  const uint height);

template <typename FrameT>
viam::sdk::Camera::raw_image encodeFrameToResponse(FrameT const &frame);

template <>
viam::sdk::Camera::raw_image
encodeFrameToResponse<rs2::video_frame>(rs2::video_frame const &frame);

template <>
viam::sdk::Camera::raw_image
encodeFrameToResponse<rs2::depth_frame>(rs2::depth_frame const &frame);

std::vector<std::uint8_t>
encodeRGBPointsToPCD(std::pair<rs2::points, rs2::video_frame> &&data);

} // namespace encoding
} // namespace realsense