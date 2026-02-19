#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "time.hpp"

using namespace realsense::time;

class TimeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Set up test time values
    base_time_ms_ = 1000.0;
    later_time_ms_ = 1500.0;
    much_later_time_ms_ = 2000.0;
    max_age_ms_ = 400.0;
    test_error_msg_ = "Test error message";
  }

  double base_time_ms_;
  double later_time_ms_;
  double much_later_time_ms_;
  double max_age_ms_;
  std::string test_error_msg_;
};

TEST_F(TimeTest, GetNowMs_ReturnsPositiveValue) {
  auto now_ms = getNowMs();

  EXPECT_GT(now_ms, 0.0);
  EXPECT_LT(now_ms, 1e20);  // Sanity check - should be reasonable timestamp
}

TEST_F(TimeTest, GetNowMs_IsIncreasing) {
  auto time1 = getNowMs();

  // Sleep for a small amount to ensure time passes
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  auto time2 = getNowMs();

  EXPECT_GT(time2, time1);
  EXPECT_LT(time2 - time1, 100.0);  // Should be small difference (< 100ms)
}

TEST_F(TimeTest, GetNowMs_MultipleCalls) {
  std::vector<double> timestamps;

  // Take multiple timestamps
  for (int i = 0; i < 5; ++i) {
    timestamps.push_back(getNowMs());
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // All timestamps should be increasing
  for (size_t i = 1; i < timestamps.size(); ++i) {
    EXPECT_GT(timestamps[i], timestamps[i - 1]);
  }
}

TEST_F(TimeTest, TimeSincePrevMs_PositiveDifference) {
  auto diff = timeSincePrevMs(later_time_ms_, base_time_ms_);

  EXPECT_EQ(diff, 500.0);  // 1500 - 1000 = 500
}

TEST_F(TimeTest, TimeSincePrevMs_ZeroDifference) {
  auto diff = timeSincePrevMs(base_time_ms_, base_time_ms_);

  EXPECT_EQ(diff, 0.0);
}

TEST_F(TimeTest, TimeSincePrevMs_NegativeDifference) {
  // When now < prev (clock went backwards), should return 0
  auto diff = timeSincePrevMs(base_time_ms_, later_time_ms_);

  EXPECT_EQ(diff, 0.0);
}

TEST_F(TimeTest, TimeSincePrevMs_EdgeCases) {
  // Test with very small differences
  auto diff1 = timeSincePrevMs(1000.001, 1000.0);
  EXPECT_NEAR(diff1, 0.001, 1e-6);

  // Test with large differences
  auto diff2 = timeSincePrevMs(1000000.0, 0.0);
  EXPECT_DOUBLE_EQ(diff2, 1000000.0);

  // Test with floating point precision
  auto diff3 = timeSincePrevMs(1000.123456789, 1000.123456788);
  EXPECT_GT(diff3, 0.0);
  EXPECT_LT(diff3, 0.01);
}

TEST_F(TimeTest, IsTooOld_WithinMaxAge) {
  // 500ms difference, max age 400ms -> should be too old
  bool result = isTooOld(later_time_ms_, base_time_ms_, max_age_ms_);

  EXPECT_TRUE(result);
}

TEST_F(TimeTest, IsTooOld_ExactlyMaxAge) {
  // Exactly at max age boundary
  bool result = isTooOld(base_time_ms_ + max_age_ms_, base_time_ms_, max_age_ms_);

  EXPECT_FALSE(result);  // Should be false when exactly equal
}

TEST_F(TimeTest, IsTooOld_JustOverMaxAge) {
  // Just over max age boundary
  bool result = isTooOld(base_time_ms_ + max_age_ms_ + 0.1, base_time_ms_, max_age_ms_);

  EXPECT_TRUE(result);
}

TEST_F(TimeTest, IsTooOld_WellWithinMaxAge) {
  // Well within max age
  bool result = isTooOld(base_time_ms_ + 100.0, base_time_ms_, max_age_ms_);

  EXPECT_FALSE(result);
}

TEST_F(TimeTest, IsTooOld_ClockWentBackwards) {
  // When clock goes backwards, timeSincePrevMs returns 0, so not too old
  bool result = isTooOld(base_time_ms_, later_time_ms_, max_age_ms_);

  EXPECT_FALSE(result);
}

TEST_F(TimeTest, IsTooOld_ZeroMaxAge) {
  // With zero max age, anything > 0 should be too old
  bool result1 = isTooOld(base_time_ms_ + 0.1, base_time_ms_, 0.0);
  EXPECT_TRUE(result1);

  bool result2 = isTooOld(base_time_ms_, base_time_ms_, 0.0);
  EXPECT_FALSE(result2);
}

TEST_F(TimeTest, LogIfTooOld_LogsWhenTooOld) {
  std::ostringstream log_stream;

  // This should trigger logging (500ms > 400ms max age)
  auto& result =
      logIfTooOld(log_stream, later_time_ms_, base_time_ms_, max_age_ms_, test_error_msg_);

  // Should return reference to the same stream
  EXPECT_EQ(&result, &log_stream);

  // Check log contents
  std::string logged = log_stream.str();
  EXPECT_THAT(logged, ::testing::HasSubstr(test_error_msg_));
  EXPECT_THAT(logged, ::testing::HasSubstr("timestamp: 1000"));
  EXPECT_THAT(logged, ::testing::HasSubstr("time diff: 500"));
}

TEST_F(TimeTest, LogIfTooOld_DoesNotLogWhenNotTooOld) {
  std::ostringstream log_stream;

  // This should NOT trigger logging (100ms < 400ms max age)
  auto& result =
      logIfTooOld(log_stream, base_time_ms_ + 100.0, base_time_ms_, max_age_ms_, test_error_msg_);

  // Should return reference to the same stream
  EXPECT_EQ(&result, &log_stream);

  // Log should be empty
  EXPECT_TRUE(log_stream.str().empty());
}

TEST_F(TimeTest, LogIfTooOld_FormatsMessageCorrectly) {
  std::ostringstream log_stream;

  logIfTooOld(log_stream, 2500.0, 1234.5, 500.0, "Frame too old");

  std::string logged = log_stream.str();
  EXPECT_THAT(logged, ::testing::HasSubstr("Frame too old"));
  EXPECT_THAT(logged, ::testing::HasSubstr("timestamp: 1234.5ms"));
  EXPECT_THAT(logged, ::testing::HasSubstr("time diff: 1265.5ms"));
}

TEST_F(TimeTest, LogIfTooOld_WorksWithDifferentSinkTypes) {
  // Test with std::stringstream
  std::stringstream string_stream;
  logIfTooOld(string_stream, later_time_ms_, base_time_ms_, max_age_ms_, "Test");
  EXPECT_FALSE(string_stream.str().empty());

  // Test with std::ostringstream
  std::ostringstream ostring_stream;
  logIfTooOld(ostring_stream, later_time_ms_, base_time_ms_, max_age_ms_, "Test");
  EXPECT_FALSE(ostring_stream.str().empty());
}

TEST_F(TimeTest, ThrowIfTooOld_ThrowsWhenTooOld) {
  EXPECT_THROW(
      { throwIfTooOld(later_time_ms_, base_time_ms_, max_age_ms_, test_error_msg_); },
      std::invalid_argument);
}

TEST_F(TimeTest, ThrowIfTooOld_DoesNotThrowWhenNotTooOld) {
  EXPECT_NO_THROW(
      { throwIfTooOld(base_time_ms_ + 100.0, base_time_ms_, max_age_ms_, test_error_msg_); });
}

TEST_F(TimeTest, ThrowIfTooOld_ExceptionMessageFormat) {
  try {
    throwIfTooOld(much_later_time_ms_, base_time_ms_, max_age_ms_, "Custom error");
    FAIL() << "Expected std::invalid_argument exception";
  } catch (const std::invalid_argument& e) {
    std::string message = e.what();
    EXPECT_THAT(message, ::testing::HasSubstr("Custom error"));
    EXPECT_THAT(message, ::testing::HasSubstr("timestamp: 1000ms"));
    EXPECT_THAT(message, ::testing::HasSubstr("time diff: 1000ms"));
  }
}

TEST_F(TimeTest, ThrowIfTooOld_EdgeCaseBoundary) {
  // Should not throw when exactly at boundary
  EXPECT_NO_THROW(
      { throwIfTooOld(base_time_ms_ + max_age_ms_, base_time_ms_, max_age_ms_, "Error"); });

  // Should throw when just over boundary
  EXPECT_THROW(
      { throwIfTooOld(base_time_ms_ + max_age_ms_ + 0.001, base_time_ms_, max_age_ms_, "Error"); },
      std::invalid_argument);
}

TEST_F(TimeTest, IntegrationTest_RealTimeScenario) {
  auto start_time = getNowMs();

  // Sleep for known duration
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  auto current_time = getNowMs();

  // Should not be too old with reasonable max age
  EXPECT_FALSE(isTooOld(current_time, start_time, 200.0));

  // Should be too old with very small max age
  EXPECT_TRUE(isTooOld(current_time, start_time, 1.0));

  // Time difference should be reasonable
  auto diff = timeSincePrevMs(current_time, start_time);
  EXPECT_GT(diff, 5.0);    // At least 5ms (we slept for 10ms)
  EXPECT_LT(diff, 100.0);  // But not more than 100ms
}

TEST_F(TimeTest, IntegrationTest_ConsistentBehaviorAcrossFunctions) {
  double now = 5000.0;
  double prev = 4500.0;
  double max_age = 400.0;

  // All functions should agree on whether timestamp is too old
  bool is_too_old = isTooOld(now, prev, max_age);
  EXPECT_TRUE(is_too_old);  // 500ms > 400ms

  // logIfTooOld should log when isTooOld returns true
  std::ostringstream log;
  logIfTooOld(log, now, prev, max_age, "Test");
  EXPECT_FALSE(log.str().empty());

  // throwIfTooOld should throw when isTooOld returns true
  EXPECT_THROW({ throwIfTooOld(now, prev, max_age, "Test"); }, std::invalid_argument);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
