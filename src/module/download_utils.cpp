#include "download_utils.hpp"
#include <curl/curl.h>
#include <stdexcept>

namespace viam {
namespace realsense {
namespace download_utils {

// Static callback function for curl to write downloaded data
static size_t curlWriteCallback(void *contents, size_t size, size_t nmemb,
                                void *userp) {
  size_t total_size = size * nmemb;
  std::vector<uint8_t> *buffer = static_cast<std::vector<uint8_t> *>(userp);
  buffer->insert(buffer->end(), static_cast<uint8_t *>(contents),
                 static_cast<uint8_t *>(contents) + total_size);
  return total_size;
}

std::vector<uint8_t> downloadFirmwareFromURL(const std::string &url,
                                             viam::sdk::LogSource &logger) {
  std::vector<uint8_t> firmware_data;

  // Initialize curl
  CURL *curl = curl_easy_init();
  if (!curl) {
    throw std::runtime_error("Failed to initialize curl");
  }

  // Set curl options
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &firmware_data);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 300L); // 5 minute timeout

  // Perform the download
  CURLcode res = curl_easy_perform(curl);

  // Check for errors
  if (res != CURLE_OK) {
    std::string error_msg = "Failed to download firmware: ";
    error_msg += curl_easy_strerror(res);
    curl_easy_cleanup(curl);
    VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
    throw std::runtime_error(error_msg);
  }

  // Check HTTP response code
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);

  if (http_code != 200) {
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[downloadFirmwareFromURL] HTTP error " << http_code
        << " when downloading firmware";
    throw std::runtime_error("HTTP error " + std::to_string(http_code) +
                             " when downloading firmware");
  }

  if (firmware_data.empty()) {
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[downloadFirmwareFromURL] Downloaded firmware file is empty";
    throw std::runtime_error("Downloaded firmware file is empty");
  }
  VIAM_SDK_LOG_IMPL(logger, info)
      << "[downloadFirmwareFromURL] Firmware downloaded successfully";

  return firmware_data;
}

} // namespace download_utils
} // namespace realsense
} // namespace viam
