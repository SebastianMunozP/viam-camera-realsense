#include "firmware_update.hpp"
#include "download_utils.hpp"
#include "zip_utils.hpp"
#include <optional>
#include <stdexcept>
#include <thread>

namespace viam {
namespace realsense {
namespace firmware_update {

std::vector<uint8_t> readFirmwareFile(const std::string &file_path) {
  // Creating a file stream to read the firmware file,
  // std::ios::binary | std::ios::ate
  // std::ios::binary: Opens the file in binary mode, which means that the file
  // is read as a sequence of bytes without any translation of special
  // characters. std::ios::ate: Opens the file at the end of the file, which
  // means that the file pointer is positioned at the end of the file, this is
  // done to calculate its size easily with only one tellg operation.
  std::ifstream file(file_path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open firmware file: " + file_path);
  }

  // Get the size of the file
  std::streamsize size = file.tellg();
  // Move the file pointer to the beginning of the file
  file.seekg(0, std::ios::beg);

  // Create a vector of bytes to store the firmware data
  std::vector<uint8_t> firmware_data(size);
  // Read the firmware data from the file
  if (!file.read(reinterpret_cast<char *>(firmware_data.data()), size)) {
    throw std::runtime_error("Failed to read firmware file: " + file_path);
  }

  return firmware_data;
}

// Get firmware download URL for a given version
std::optional<std::string>
getFirmwareURLForVersion(const std::string &version) {
  // Mapping of known firmware versions to download URLs
  static const std::unordered_map<std::string, std::string> firmware_url_map = {
      {"5.17.0.10", "https://realsenseai.com/wp-content/uploads/2025/07/"
                    "d400_series_production_fw_5_17_0_10.zip"},
      // Add more firmware versions here as they become available
  };

  auto it = firmware_url_map.find(version);
  if (it != firmware_url_map.end()) {
    return it->second;
  }
  return std::nullopt;
}

} // namespace firmware_update
} // namespace realsense
} // namespace viam
