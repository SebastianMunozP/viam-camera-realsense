#pragma once

#ifdef ENABLE_PROFILING
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <gperftools/profiler.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace realsense {
namespace profiling {

class Profiler {
private:
  std::string camera_name_;
  std::string serial_number_;
  std::string profile_dir_ = "/tmp/realsense_profiles";
  std::string cpu_profile_path_;
  bool cpu_profiling_started_ = false;

public:
  Profiler(const std::string &camera_name, const std::string &serial_number)
      : camera_name_(camera_name), serial_number_(serial_number) {

    // Check environment variables
    if (const char *env_dir = std::getenv("PROFILE_OUTPUT_DIR")) {
      profile_dir_ = env_dir;
    }

    // Create camera-specific profile directory
    profile_dir_ = profile_dir_ + "/" + serial_number_;
    system(("mkdir -p " + profile_dir_).c_str());

    std::cout << "[" << serial_number_
              << "] Profiler initialized for: " << camera_name_ << std::endl;
    std::cout << "[" << serial_number_
              << "] Profile directory: " << profile_dir_ << std::endl;

    start_session_profiling();
  }

  ~Profiler() { stop_session_profiling(); }

  bool is_enabled() const { return true; }

  void log_operation_start(const std::string &operation) {
    if (!is_enabled())
      return;
    std::cout << "[" << serial_number_ << "][PROFILE] Starting: " << operation
              << std::endl;
  }

  void log_operation_end(const std::string &operation,
                         std::chrono::microseconds duration) {
    if (!is_enabled())
      return;
    std::cout << "[" << serial_number_ << "][PROFILE] Completed: " << operation
              << " (" << duration.count() << "μs)" << std::endl;
  }

private:
  void start_session_profiling() {

    auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();

    if (!cpu_profiling_started_) {
      cpu_profile_path_ =
          profile_dir_ + "/cpu_session_" + std::to_string(timestamp) + ".prof";
      std::cout << "[" << serial_number_
                << "] Starting CPU profiling session: " << cpu_profile_path_
                << std::endl;

      if (ProfilerStart(cpu_profile_path_.c_str()) != 0) {
        cpu_profiling_started_ = true;
        std::cout << "[" << serial_number_
                  << "] CPU profiling session started successfully"
                  << std::endl;
      } else {
        std::cerr << "[" << serial_number_ << "] Failed to start CPU profiler!"
                  << std::endl;
      }
    }
  }

  void stop_session_profiling() {
    if (cpu_profiling_started_) {
      std::cout << "[" << serial_number_
                << "] Stopping CPU profiling session..." << std::endl;
      ProfilerFlush();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ProfilerStop();
      cpu_profiling_started_ = false;

      // Check file size
      check_profile_file(cpu_profile_path_, "CPU session");
    }
  }

  void check_profile_file(const std::string &file_path,
                          const std::string &type) {
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (file.is_open()) {
      auto size = file.tellg();
      if (size > 0) {
        std::cout << "[" << serial_number_ << "] " << type
                  << " profile: " << size << " bytes" << std::endl;
        std::cout << "[" << serial_number_
                  << "] Analyze with: pprof --text ./viam-camera-realsense "
                  << file_path << std::endl;
      } else {
        std::cout << "[" << serial_number_ << "] Warning: " << type
                  << " profile file is empty" << std::endl;
      }
      file.close();
    } else {
      std::cout << "[" << serial_number_ << "] Warning: " << type
                << " profile file not found" << std::endl;
    }
  }
};

// RAII profiling scope for operation timing
class ProfileScope {
private:
  std::string operation_;
  std::chrono::high_resolution_clock::time_point start_time_;
  Profiler *profiler_;

public:
  ProfileScope(Profiler *profiler, const std::string &operation)
      : operation_(operation), profiler_(profiler) {
    if (profiler_ && profiler_->is_enabled()) {
      start_time_ = std::chrono::high_resolution_clock::now();
      profiler_->log_operation_start(operation_);
      ProfilerRegisterThread(); // Ensure this thread is registered with
                                // profiler
    }
  }

  ~ProfileScope() {
    if (profiler_ && profiler_->is_enabled()) {
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          end_time - start_time_);
      profiler_->log_operation_end(operation_, duration);
    }
  }
};

} // namespace profiling
} // namespace realsense

// Convenience macros
#define CAMERA_PROFILE_SCOPE(profiler, name)                                   \
  realsense::profiling::ProfileScope _prof_scope(profiler, name)

#else // !ENABLE_PROFILING

namespace realsense {
namespace profiling {

// No-op profiler when profiling is disabled
class Profiler {
public:
  Profiler(const std::string &, const std::string &) {}
  bool is_enabled() const { return false; }
  void log_operation_start(const std::string &) {}
  void log_operation_end(const std::string &, std::chrono::microseconds) {}
};

class ProfileScope {
public:
  ProfileScope(Profiler *, const std::string &) {}
};

} // namespace profiling
} // namespace realsense

#define CAMERA_PROFILE_SCOPE(profiler, name)                                   \
  do {                                                                         \
  } while (0)

#endif // ENABLE_PROFILING
