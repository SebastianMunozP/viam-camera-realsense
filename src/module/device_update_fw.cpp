#include <curl/curl.h>
#include <math.h>
#include <zip.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace realsense {

size_t writeFileCallback(void *contents, size_t size, size_t nmemb,
                         void *userp) {
  auto *buffer = static_cast<std::vector<char> *>(userp);
  size_t totalSize = size * nmemb;
  buffer->insert(buffer->end(), static_cast<char *>(contents),
                 static_cast<char *>(contents) + totalSize);
  return totalSize;
}

std::vector<std::uint8_t> downloadFirmware(std::string firmwareUrl) {
  CleanupPtr<curl_easy_cleanup> curl(curl_easy_init());
  if (!curl) {
    throw std::invalid_argument("curl easy init failed");
  }

  // Download the firmware and write it to a buffer
  std::vector<std::uint8_t> zipBuffer;
  curl_easy_setopt(curl.get(), CURLOPT_URL, firmwareUrl.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, writeFileCallback);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &zipBuffer);
  CURLcode res = curl_easy_perform(curl.get());
  if (res != CURLE_OK) {
    std::ostringstream buffer;
    buffer << "curl early perform failed: " << curl_easy_strerror(res);
    throw std::invalid_argument(buffer.str());
  }

  std::vector<uint8_t> binData;
  zip_error_t ziperror;
  zip_error_init(&ziperror);

  zip_source_t *src = zip_source_buffer_create(zipBuffer.data(),
                                               zipBuffer.size(), 0, &ziperror);
  if (!src) {
    std::ostringstream buffer;
    buffer << "failed to create zip buffer: " << zip_error_strerror(&ziperror);
    throw std::runtime_error(buffer.str());
  }

  // Ensure src cleanup if zip_open fails
  CleanupPtr<zip_source_free> srcCleanup(src);

  // If this succeeds, zip takes ownership of src, so src will be freed when
  // zip_close is called.
  zip_t *zip = zip_open_from_source(src, 0, &ziperror);
  if (!zip) {
    std::ostringstream buffer;
    buffer << "failed to open zip from source: "
           << zip_error_strerror(&ziperror);
    throw std::runtime_error(buffer.str());
  }

  srcCleanup.release();
  CleanupPtr<zip_close> zipCleanup(zip);

  if (zip_get_num_entries(zip, 0) != 1) {
    throw std::runtime_error("unexpected number of files in firmware zip");
  }

  const char *fileName = zip_get_name(zip, 0, 0);
  if (!fileName) {
    throw std::runtime_error("couldn't get bin file name");
  }

  CleanupPtr<zip_fclose> binFile(zip_fopen(zip, fileName, 0));
  if (!binFile) {
    throw std::runtime_error("failed to open the firmware bin file");
  }

  zip_stat_t stats;
  zip_stat_init(&stats);
  if (zip_stat(zip, fileName, 0, &stats) != 0) {
    throw std::invalid_argument("failed to stat file");
  }

  binData.resize(stats.size);
  zip_int64_t bytesRead = zip_fread(binFile.get(), binData.data(), stats.size);
  if (bytesRead == -1) {
    zip_error_t *err = zip_file_get_error(binFile.get());
    std::ostringstream buffer;
    buffer << "failed to read bin: " << zip_error_strerror(err);
    throw std::runtime_error(buffer.str());
  }

  if (bytesRead != stats.size) {
    std::ostringstream buffer;
    buffer << "failed to fully read binary file, file size: " << stats.size
           << "bytes read: " << bytesRead;
    throw std::runtime_error(buffer.str());
  }
}

} // namespace realsense
