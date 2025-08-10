
#pragma once
#include "utils.hpp"
#include <turbojpeg.h>
namespace realsense {
namespace encoding {

std::vector<std::uint8_t> encodeDepthRAW(const std::uint8_t* data, const uint64_t width,
                                const uint64_t height) {
    viam::sdk::Camera::depth_map m = xt::xarray<uint16_t>::from_shape({height, width});
    std::copy(reinterpret_cast<const uint16_t*>(data),
              reinterpret_cast<const uint16_t*>(data) + height * width, m.begin());

    return viam::sdk::Camera::encode_depth_map(m);
}

viam::sdk::Camera::raw_image encodeDepthRAWToResponse(const std::uint8_t* data,
                                                                       const uint width,
                                                                       const uint height) {
    viam::sdk::Camera::raw_image response{};
    response.source_name = "depth";
    response.mime_type = "image/vnd.viam.dep";
    response.bytes = encodeDepthRAW(data, width, height);
    return response;
}

std::vector<std::uint8_t> encodeJPEG(const std::uint8_t * data, const uint width, const uint height) {
    std::uint8_t* encoded = nullptr;
    std::size_t encodedSize = 0;
    tjhandle handle = tjInitCompress();
    if (handle == nullptr) {
        throw std::runtime_error("[GetImage] failed to init JPEG compressor");
    }
    int success;
    try {
        success = tjCompress2(handle, data, width, 0, height, TJPF_RGB, &encoded, &encodedSize,
                              TJSAMP_420, 75, TJFLAG_FASTDCT);
        tjDestroy(handle);
    } catch (const std::exception& e) {
        tjDestroy(handle);
        throw std::runtime_error("[GetImage] JPEG compressor failed to compress: " +
                                 std::string(e.what()));
    }
    if (success != 0) {
        throw std::runtime_error("[GetImage] JPEG compressor failed to compress image");
    }

    std::vector<std::uint8_t> output(encoded, encoded + encodedSize);
    return output;
}


viam::sdk::Camera::raw_image encodeJPEGToResponse(const std::uint8_t * data,
                                                                   const uint width,
                                                                   const uint height) {
    viam::sdk::Camera::raw_image response;
    response.source_name = "color";
    response.mime_type = "image/jpeg";
    response.bytes = encodeJPEG(data, width, height);
    return response;
}

template <typename FrameT>
viam::sdk::Camera::raw_image encodeResponse(FrameT const& frame) {
    VIAM_SDK_LOG(error) << "[get_image] unsupported frame type";
    return {};
}

template<>
viam::sdk::Camera::raw_image encodeResponse<rs2::video_frame>(rs2::video_frame const& frame) {
    auto data = reinterpret_cast<const std::uint8_t *>(frame.get_data());
    if (data == nullptr) {
        throw std::runtime_error("[get_image] frame data is null");
    }

    auto stream_profile = frame.get_profile().as<rs2::video_stream_profile>();
    if (not stream_profile) {
        throw std::runtime_error("[get_image] frame profile is not a video stream profile");
    }
    return encodeJPEGToResponse(data,
                                              stream_profile.width(), stream_profile.height());
}

template<>
viam::sdk::Camera::raw_image encodeResponse<rs2::depth_frame>(rs2::depth_frame const& frame) {
    auto data = reinterpret_cast<const std::uint8_t *>(frame.get_data());
    if (data == nullptr) {
        throw std::runtime_error("[get_image] frame data is null");
    }

    auto stream_profile = frame.get_profile().as<rs2::video_stream_profile>();
    if (not stream_profile) {
        throw std::runtime_error("[get_image] frame profile is not a video stream profile");
    }
    return encodeDepthRAWToResponse(data,
                                              stream_profile.width(), stream_profile.height());
}

template <typename FrameT>
viam::sdk::Camera::raw_image encodeFrameToResponse(FrameT const& frame) {
            BOOST_ASSERT_MSG(frame, "[encodeFrameToResponse] frame is invalid");

            // If the image's timestamp is older than a second throw error, this
            // indicates we no longer have a working camera.
            double nowMs = utils::getNowMs();
            double frame_time = frame.get_timestamp();
            uint64_t timeElapsedSinceLastFrameMs = utils::timeSinceFrameMs(nowMs, frame_time);
            BOOST_ASSERT_MSG(timeElapsedSinceLastFrameMs <= utils::maxFrameAgeMs, "[encodeFrameToResponse] frame is too old");

            return encoding::encodeResponse(frame);
}


}
}