#pragma once
#include <chrono>  // For std::chrono
#include <cstdint> // For std::uint64_t
#include <sstream>
#include <string>
namespace realsense {
namespace utils {
inline double getNowMs() {
  auto now = std::chrono::high_resolution_clock::now();
  auto now_ms =
      std::chrono::duration<double, std::milli>(now.time_since_epoch())
          .count(); // System time (ms)

  return now_ms;
}

inline std::uint64_t timeSincePrevMs(double nowMs, double prevTimeMs) {
  if (nowMs > prevTimeMs) {
    return nowMs - prevTimeMs;
  }
  return 0;
}

inline bool isTooOld(double const nowMs, double const prevTimeMs,
                     double const maxAgeMs) {
  return timeSincePrevMs(nowMs, prevTimeMs) > maxAgeMs;
}

template <typename SinkT>
SinkT &logIfTooOld(SinkT &sink, double const nowMs, double const prevTimeMs,
                   double const maxAgeMs, std::string const &error_msg) {
  if (isTooOld(nowMs, prevTimeMs, maxAgeMs)) {
    sink << error_msg << ", timestamp: " << prevTimeMs
         << "ms, time diff: " << timeSincePrevMs(nowMs, prevTimeMs) << "ms";
  }
  return sink;
}

inline void throwIfTooOld(double const nowMs, double const prevTimeMs,
                          double const maxAgeMs, std::string const &error_msg) {
  if (isTooOld(nowMs, prevTimeMs, maxAgeMs)) {
    std::ostringstream buffer;
    buffer << error_msg << ", timestamp: " << prevTimeMs
           << "ms, time diff: " << timeSincePrevMs(nowMs, prevTimeMs) << "ms";
    throw std::invalid_argument(buffer.str());
  }
}

} // namespace utils
} // namespace realsense