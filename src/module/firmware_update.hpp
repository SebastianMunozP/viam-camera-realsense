#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace viam {
namespace realsense {

/**
 * Read firmware data from a local file.
 * @param file_path Path to the firmware file
 * @return The firmware data as a vector of bytes
 * @throws std::runtime_error if file cannot be opened or read
 */
std::vector<uint8_t> readFirmwareFile(const std::string &file_path);

} // namespace realsense
} // namespace viam
