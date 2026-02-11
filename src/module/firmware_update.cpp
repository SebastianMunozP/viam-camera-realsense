#include "firmware_update.hpp"
#include <stdexcept>

namespace viam {
namespace realsense {

std::vector<uint8_t> readFirmwareFile(const std::string &file_path) {
  std::ifstream file(file_path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open firmware file: " + file_path);
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<uint8_t> firmware_data(size);
  if (!file.read(reinterpret_cast<char *>(firmware_data.data()), size)) {
    throw std::runtime_error("Failed to read firmware file: " + file_path);
  }

  return firmware_data;
}

} // namespace realsense
} // namespace viam
