#pragma once

#ifdef ENABLE_PROFILING
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <gperftools/heap-profiler.h>
#include <gperftools/profiler.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace realsense {
namespace profiling {

class CameraProfiler {
private:
  std::string camera_name_;
  std::string serial_number_;
  bool cpu_profiling_enabled_ = false;
  bool heap_profiling_enabled_ = false;
  std::string profile_dir_ = "/tmp/realsense_profiles";
  std::string cpu_profile_path_;
  std::string heap_profile_path_;
  bool cpu_profiling_started_ = false;
  bool heap_profiling_started_ = false;

public:
  CameraProfiler(const std::string &camera_name,
                 const std::string &serial_number)
      : camera_name_(camera_name), serial_number_(serial_number) {

    // Check environment variables
    if (const char *env_cpu = std::getenv("ENABLE_CPU_PROFILING")) {
      cpu_profiling_enabled_ = (std::string(env_cpu) == "1");
    }

    if (const char *env_heap = std::getenv("ENABLE_HEAP_PROFILING")) {
      heap_profiling_enabled_ = (std::string(env_heap) == "1");
    }

    if (const char *env_dir = std::getenv("PROFILE_OUTPUT_DIR")) {
      profile_dir_ = env_dir;
    }

    // Create camera-specific profile directory
    profile_dir_ = profile_dir_ + "/" + serial_number_;
    system(("mkdir -p " + profile_dir_).c_str());

    if (cpu_profiling_enabled_ || heap_profiling_enabled_) {
      std::cout << "[" << serial_number_
                << "] CameraProfiler initialized for: " << camera_name_
                << std::endl;
      std::cout << "[" << serial_number_ << "] CPU profiling: "
                << (cpu_profiling_enabled_ ? "ENABLED" : "DISABLED")
                << std::endl;
      std::cout << "[" << serial_number_ << "] Heap profiling: "
                << (heap_profiling_enabled_ ? "ENABLED" : "DISABLED")
                << std::endl;
      std::cout << "[" << serial_number_
                << "] Profile directory: " << profile_dir_ << std::endl;
      start_session_profiling();
    }
  }

  ~CameraProfiler() { stop_session_profiling(); }

  bool is_enabled() const {
    return cpu_profiling_enabled_ || heap_profiling_enabled_;
  }

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
    if (!cpu_profiling_enabled_ && !heap_profiling_enabled_)
      return;

    auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();

    if (cpu_profiling_enabled_ && !cpu_profiling_started_) {
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
        cpu_profiling_enabled_ = false;
      }
    }

    if (heap_profiling_enabled_ && !heap_profiling_started_) {
      heap_profile_path_ =
          profile_dir_ + "/heap_session_" + std::to_string(timestamp) + ".heap";
      std::cout << "[" << serial_number_
                << "] Starting heap profiling session: " << heap_profile_path_
                << std::endl;
      HeapProfilerStart(heap_profile_path_.c_str());
      heap_profiling_started_ = true;
      std::cout << "[" << serial_number_
                << "] Heap profiling session started successfully" << std::endl;
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

    if (heap_profiling_started_) {
      std::cout << "[" << serial_number_
                << "] Stopping heap profiling session..." << std::endl;
      HeapProfilerDump("final");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      HeapProfilerStop();
      heap_profiling_started_ = false;

      // Check file size
      check_profile_file(heap_profile_path_, "Heap session");
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
  CameraProfiler *profiler_;

public:
  ProfileScope(CameraProfiler *profiler, const std::string &operation)
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
class CameraProfiler {
public:
  CameraProfiler(const std::string &, const std::string &) {}
  bool is_enabled() const { return false; }
  void log_operation_start(const std::string &) {}
  void log_operation_end(const std::string &, std::chrono::microseconds) {}
};

class ProfileScope {
public:
  ProfileScope(CameraProfiler *, const std::string &) {}
};

} // namespace profiling
} // namespace realsense

#define CAMERA_PROFILE_SCOPE(profiler, name)                                   \
  do {                                                                         \
  } while (0)

#endif // ENABLE_PROFILING
