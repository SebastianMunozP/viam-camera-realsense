#pragma once
namespace realsense {
namespace utils {
    static const uint64_t maxFrameAgeMs = 1e3; // time until a frame is considered stale, in miliseconds (equal to 1 sec)
    inline double getNowMs()
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto now_ms = std::chrono::duration<double, std::milli>(now.time_since_epoch()).count(); // System time (ms)

        return now_ms;
    }

    inline uint64_t timeSinceFrameMs(double nowMs, double imageTimeMs)
    {
        if (nowMs > imageTimeMs)
        {
            return nowMs - imageTimeMs;
        }
        return 0;
    }
    
}
}