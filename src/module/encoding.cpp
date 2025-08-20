#include "encoding.hpp"
#include <turbojpeg.h>

namespace realsense {
namespace encoding {

std::vector<std::uint8_t> encodeDepthRAW(const std::uint8_t *data,
                                         const uint64_t width,
                                         const uint64_t height) {
  viam::sdk::Camera::depth_map m =
      xt::xarray<uint16_t>::from_shape({height, width});
  std::copy(reinterpret_cast<const uint16_t *>(data),
            reinterpret_cast<const uint16_t *>(data) + height * width,
            m.begin());

  return viam::sdk::Camera::encode_depth_map(m);
}

viam::sdk::Camera::raw_image encodeDepthRAWToResponse(const std::uint8_t *data,
                                                      const uint width,
                                                      const uint height) {
  viam::sdk::Camera::raw_image response{};
  response.source_name = "depth";
  response.mime_type = "image/vnd.viam.dep";
  response.bytes = encodeDepthRAW(data, width, height);
  return response;
}

std::vector<std::uint8_t> encodeJPEG(const std::uint8_t *data, const uint width,
                                     const uint height) {
  std::uint8_t *encoded = nullptr;
  std::size_t encodedSize = 0;
  tjhandle handle = tjInitCompress();
  if (handle == nullptr) {
    throw std::runtime_error("[encodeJPEG] failed to init JPEG compressor");
  }
  int success;
  try {
    success = tjCompress2(handle, data, width, 0, height, TJPF_RGB, &encoded,
                          &encodedSize, TJSAMP_420, 75, TJFLAG_FASTDCT);
    tjDestroy(handle);
  } catch (const std::exception &e) {
    tjDestroy(handle);
    throw std::runtime_error(
        "[encodeJPEG] JPEG compressor failed to compress: " +
        std::string(e.what()));
  }
  if (success != 0) {
    throw std::runtime_error(
        "[encodeJPEG] JPEG compressor failed to compress image");
  }

  std::vector<std::uint8_t> output(encoded, encoded + encodedSize);
  return output;
}

viam::sdk::Camera::raw_image encodeJPEGToResponse(const std::uint8_t *data,
                                                  const uint width,
                                                  const uint height) {
  viam::sdk::Camera::raw_image response;
  response.source_name = "color";
  response.mime_type = "image/jpeg";
  response.bytes = encodeJPEG(data, width, height);
  return response;
}
// Template specializations
template <typename FrameT>
viam::sdk::Camera::raw_image encodeFrameToResponse(FrameT const &frame) {
  VIAM_SDK_LOG(error) << "[encodeFrameToResponse] unsupported frame type";
  return {};
}
template <>
viam::sdk::Camera::raw_image
encodeFrameToResponse<rs2::video_frame>(rs2::video_frame const &frame) {
  auto data = reinterpret_cast<const std::uint8_t *>(frame.get_data());
  if (data == nullptr) {
    throw std::runtime_error("[encodeFrameToResponse] frame data is null");
  }

  auto stream_profile = frame.get_profile().as<rs2::video_stream_profile>();
  if (not stream_profile) {
    throw std::runtime_error(
        "[encodeFrameToResponse] frame profile is not a video stream profile");
  }
  return encodeJPEGToResponse(data, stream_profile.width(),
                              stream_profile.height());
}

template <>
viam::sdk::Camera::raw_image
encodeFrameToResponse<rs2::depth_frame>(rs2::depth_frame const &frame) {
  auto data = reinterpret_cast<const std::uint8_t *>(frame.get_data());
  if (data == nullptr) {
    throw std::runtime_error("[encodeFrameToResponse] frame data is null");
  }

  auto stream_profile = frame.get_profile().as<rs2::video_stream_profile>();
  if (not stream_profile) {
    throw std::runtime_error(
        "[encodeFrameToResponse] frame profile is not a video stream profile");
  }
  return encodeDepthRAWToResponse(data, stream_profile.width(),
                                  stream_profile.height());
}

struct PointXYZRGB {
  float x, y, z;
  unsigned int rgb;
};

std::vector<PointXYZRGB>
getPCDPoints(std::pair<rs2::points, rs2::video_frame> &&data) {
  rs2::points const &points = data.first;
  rs2::video_frame const &color = data.second;

  if (not points or not color) {
    throw std::runtime_error("Points or color frame data is empty");
  }

  int num_points = points.size();
  const rs2::vertex *vertices = points.get_vertices();
  const rs2::texture_coordinate *tex_coords = points.get_texture_coordinates();

  const uint8_t *color_data =
      reinterpret_cast<const uint8_t *>(color.get_data());
  int width = color.get_width();
  int height = color.get_height();

  std::vector<PointXYZRGB> pcdPoints;
  pcdPoints.reserve(num_points);

  for (int i = 0; i < num_points; ++i) {
    auto const &vertex = vertices[i];

    auto const &tc = tex_coords[i];
    int u =
        std::clamp(static_cast<int>(std::lround(tc.u * width)), 0, width - 1);
    int v =
        std::clamp(static_cast<int>(std::lround(tc.v * height)), 0, height - 1);

    size_t idx = (v * width + u) * 3;
    auto r = static_cast<unsigned int>(color_data[idx]);
    auto g = static_cast<unsigned int>(color_data[idx + 1]);
    auto b = static_cast<unsigned int>(color_data[idx + 2]);
    std::uint32_t rgb = (r << 16) | (g << 8) | b;

    PointXYZRGB pt;
    pt.x = vertex.x;
    pt.y = vertex.y;
    pt.z = vertex.z;
    pt.rgb = rgb;

    pcdPoints.emplace_back(pt);
  }

  return pcdPoints;
}

std::vector<std::uint8_t>
encodeRGBPointsToPCD(std::pair<rs2::points, rs2::video_frame> &&data) {
  auto pcdPoints = getPCDPoints(std::move(data));
  if (pcdPoints.empty()) {
    VIAM_SDK_LOG(error) << "[encodeRGBPointsToPCD] No valid points found";
    return {};
  }

  std::stringstream header;
  header << "VERSION .7\n"
         << "FIELDS x y z rgb\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F U\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << pcdPoints.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << pcdPoints.size() << "\n"
         << "DATA binary\n";
  std::string headerStr = header.str();
  std::vector<std::uint8_t> pcdBytes;
  pcdBytes.insert(pcdBytes.end(), headerStr.begin(), headerStr.end());

  // Assert that PointXYZRGB is a POD type (a Plain Old Data type is defined by
  // being trivially copyable and having a standard memory layout), and that it
  // has no padding, thus can be copied as bytes. Since vector is contiguous, we
  // can just copy the whole thing
  static_assert(
      std::is_trivially_copyable_v<PointXYZRGB> and
          std::is_standard_layout_v<PointXYZRGB>,
      "PointXYZRGB must be trivially copyable and have standard layout");
  static_assert(sizeof(PointXYZRGB) ==
                    (3 * sizeof(float)) + sizeof(unsigned int),
                "PointXYZRGB has unexpected padding");

  const std::uint8_t *dataPtr =
      reinterpret_cast<const std::uint8_t *>(pcdPoints.data());
  size_t dataSize = pcdPoints.size() * sizeof(PointXYZRGB);
  pcdBytes.insert(pcdBytes.end(), dataPtr, dataPtr + dataSize);

  VIAM_SDK_LOG(debug) << "[encodeRGBPointsToPCD] Converted " << pcdPoints.size()
                      << " points to PCD format, encoded in " << pcdBytes.size()
                      << " bytes";
  return pcdBytes;
}

} // namespace encoding
} // namespace realsense
