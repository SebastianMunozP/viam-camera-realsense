#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/log/logging.hpp>

#include "device.hpp"
#include "log_capture.hpp"
#include "realsense.hpp"
#include "sensors.hpp"

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::StrictMock;

namespace realsense {
namespace device {
template <typename DeviceT, typename PipeT, typename AligntT, typename ConfigT>
std::ostream &
operator<<(std::ostream &os,
           const ViamRSDevice<DeviceT, PipeT, AligntT, ConfigT> &device) {
  os << "ViamRSDevice{serial: " << device.serial_number
     << ", started: " << device.started << "}";
  return os;
}
} // namespace device
} // namespace realsense

namespace realsense {
namespace device {
namespace test {

// Mock classes for testing
class MockDevice : public rs2::device {
public:
  MOCK_METHOD(bool, supports, (rs2_camera_info), (const));
  MOCK_METHOD(const char *, get_info, (rs2_camera_info), (const));
  MOCK_METHOD(std::vector<rs2::sensor>, query_sensors, (), (const));
};

class MockSensor {
public:
  MOCK_METHOD(bool, is, (), (const));
  MOCK_METHOD(std::vector<rs2::stream_profile>, get_stream_profiles, (),
              (const));

  template <typename T> bool is() const { return is(); }
};

class MockVideoStreamProfile {
public:
  MOCK_METHOD(rs2_format, format, (), (const));
  MOCK_METHOD(int, width, (), (const));
  MOCK_METHOD(int, height, (), (const));
  MOCK_METHOD(int, fps, (), (const));
  MOCK_METHOD(int, stream_index, (), (const));
};

class MockConfig {
public:
  MOCK_METHOD(void, enable_stream,
              (rs2_stream, int, int, int, rs2_format, int));
  MOCK_METHOD(void, enable_device, (const std::string &));
};

class MockPipeline {
public:
  MOCK_METHOD(void, start, (const rs2::config &));
  MOCK_METHOD(void, start,
              (const rs2::config &, std::function<void(const rs2::frame &)>));
  MOCK_METHOD(void, stop, ());
};

class MockAlign {
public:
  MockAlign(rs2_stream) {} // Constructor for align
};

// Test fixture
class DeviceTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Setup common test data
    serial_number_ = "test_device_123456";
    supported_models_ = {"D435", "D435i", "D455"};

    // Setup mock device
    mock_device_ = std::make_shared<MockDevice>();
    mock_config_ = std::make_shared<MockConfig>();
    mock_pipeline_ = std::make_shared<MockPipeline>();

    // Setup test config with proper constructor parameters
    test_config_ =
        RsResourceConfig(serial_number_, // serial_number
                         "camera1",      // name
                         std::vector<realsense::sensors::SensorType>{
                             realsense::sensors::SensorType::color,
                             realsense::sensors::SensorType::depth}, // sensors
                         std::optional<int>{640},                    // width
                         std::optional<int>{480}                     // height
        );
  }

  std::string serial_number_;
  std::unordered_set<std::string> supported_models_;
  std::shared_ptr<MockDevice> mock_device_;
  std::shared_ptr<MockConfig> mock_config_;
  std::shared_ptr<MockPipeline> mock_pipeline_;
  RsResourceConfig test_config_;
};

class RealsenseTestEnvironment : public ::testing::Environment {
public:
  void SetUp() override {
    // Create the instance here, before any tests run
    instance_ = std::make_unique<viam::sdk::Instance>();
  }

  void TearDown() override {
    // Clean up the instance
    instance_.reset();
  }

private:
  std::unique_ptr<viam::sdk::Instance> instance_;
};

// Test getCameraModel function
TEST_F(DeviceTest, GetCameraModel_ValidDevice_ReturnsModel) {
  // Setup expectations
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return("Intel RealSense D435"));

  // Execute
  auto result = getCameraModel(mock_device_);

  // Verify
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), "D435");
}

TEST_F(DeviceTest, GetCameraModel_UnsupportedDevice_ReturnsNullopt) {
  // Setup expectations
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(false));

  // Execute
  auto result = getCameraModel(mock_device_);

  // Verify
  EXPECT_FALSE(result.has_value());
}

TEST_F(DeviceTest, GetCameraModel_InvalidNameFormat_ReturnsNullopt) {
  // Setup expectations
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return("Invalid Name"));

  // Execute
  auto result = getCameraModel(mock_device_);

  // Verify
  EXPECT_FALSE(result.has_value());
}

// Test printDeviceInfo function
TEST_F(DeviceTest, PrintDeviceInfo_ValidDevice_LogsInfo) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Setup expectations for supported info types
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return("Intel RealSense D435"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))
      .WillOnce(Return("123456789"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_PRODUCT_LINE))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_PRODUCT_LINE))
      .WillOnce(Return("D400 Series"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_PRODUCT_ID))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_PRODUCT_ID))
      .WillOnce(Return("0B07"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillOnce(Return("USB 3.1"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillOnce(Return("5.12.7.100"));

  EXPECT_CALL(*mock_device_,
              supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_,
              get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillOnce(Return("5.12.7.100"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID))
      .WillOnce(Return("5.12.7.100"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_PHYSICAL_PORT))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_PHYSICAL_PORT))
      .WillOnce(Return("USB 3.1 Port"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_DEBUG_OP_CODE))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE))
      .WillOnce(Return("Debug Op Code Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_ADVANCED_MODE))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_ADVANCED_MODE))
      .WillOnce(Return("Advanced Mode Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillOnce(Return("Camera Locked Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER))
      .WillOnce(Return("ASIC Serial Number Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_DFU_DEVICE_PATH))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_DFU_DEVICE_PATH))
      .WillOnce(Return("DFU Device Path Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_IP_ADDRESS))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_IP_ADDRESS))
      .WillOnce(Return("IP Address Info"));

  // Execute - should not throw and should log device info
  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  // Verify logs were produced
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "printDeviceInfo should produce info logs";

  // Verify no errors were logged
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0) << "Should not log errors for valid device";

  // Verify device info is in the logs
  bool found_device_info = false;
  for (const auto& log : all_logs) {
    if (log.message.find("DeviceInfo") != std::string::npos ||
        log.message.find("Intel RealSense D435") != std::string::npos ||
        log.message.find("123456789") != std::string::npos) {
      found_device_info = true;
      break;
    }
  }
  EXPECT_TRUE(found_device_info) << "Should log device information";
}

// Test SensorTypeTraits
TEST_F(DeviceTest, SensorTypeTraits_ColorSensor_CorrectValues) {
  EXPECT_EQ(SensorTypeTraits<rs2::color_sensor>::stream_type, RS2_STREAM_COLOR);
  EXPECT_EQ(SensorTypeTraits<rs2::color_sensor>::format_type, RS2_FORMAT_RGB8);
}

TEST_F(DeviceTest, SensorTypeTraits_DepthSensor_CorrectValues) {
  EXPECT_EQ(SensorTypeTraits<rs2::depth_sensor>::stream_type, RS2_STREAM_DEPTH);
  EXPECT_EQ(SensorTypeTraits<rs2::depth_sensor>::format_type, RS2_FORMAT_Z16);
}

// Test checkIfMatchingColorDepthProfiles
TEST_F(DeviceTest,
       CheckIfMatchingColorDepthProfiles_MatchingProfiles_ReturnsTrue) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockVideoStreamProfile color_profile;
  MockVideoStreamProfile depth_profile;

  // Setup matching profiles
  EXPECT_CALL(color_profile, width()).WillRepeatedly(Return(640));
  EXPECT_CALL(color_profile, height()).WillRepeatedly(Return(480));
  EXPECT_CALL(color_profile, fps()).WillRepeatedly(Return(30));

  EXPECT_CALL(depth_profile, width()).WillRepeatedly(Return(640));
  EXPECT_CALL(depth_profile, height()).WillRepeatedly(Return(480));
  EXPECT_CALL(depth_profile, fps()).WillRepeatedly(Return(30));

  // Execute
  bool result =
      checkIfMatchingColorDepthProfiles(color_profile, depth_profile, logger);

  // Verify function result
  EXPECT_TRUE(result);

  // Verify info log with resolution details
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log matching profile info";

  // Verify the log contains resolution/fps info
  bool found_profile_info = false;
  for (const auto& log : all_logs) {
    if (log.message.find("640") != std::string::npos &&
        log.message.find("480") != std::string::npos &&
        log.message.find("30") != std::string::npos) {
      found_profile_info = true;
      break;
    }
  }
  EXPECT_TRUE(found_profile_info) << "Should log profile dimensions";
}

TEST_F(DeviceTest,
       CheckIfMatchingColorDepthProfiles_DifferentResolution_ReturnsFalse) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockVideoStreamProfile color_profile;
  MockVideoStreamProfile depth_profile;

  // Setup non-matching profiles
  EXPECT_CALL(color_profile, width()).WillRepeatedly(Return(640));
  EXPECT_CALL(color_profile, height()).WillRepeatedly(Return(480));
  EXPECT_CALL(color_profile, fps()).WillRepeatedly(Return(30));

  EXPECT_CALL(depth_profile, width()).WillRepeatedly(Return(1280));
  EXPECT_CALL(depth_profile, height()).WillRepeatedly(Return(720));
  EXPECT_CALL(depth_profile, fps()).WillRepeatedly(Return(30));

  // Execute
  bool result =
      checkIfMatchingColorDepthProfiles(color_profile, depth_profile, logger);

  // Verify function result
  EXPECT_FALSE(result);

  // Verify NO logs are produced for non-matching profiles
  auto all_logs = log_capture.get_records();
  EXPECT_EQ(all_logs.size(), 0) << "Should not log for non-matching profiles";
}

// Test ViamRSDevice structure
TEST_F(DeviceTest, ViamRSDevice_DefaultConstruction_ValidState) {
  ViamRSDevice<> device;

  // Default values should be reasonable
  EXPECT_TRUE(device.serial_number.empty());
  EXPECT_FALSE(device.started);
  EXPECT_EQ(device.device, nullptr);
  EXPECT_EQ(device.pipe, nullptr);
  EXPECT_EQ(device.point_cloud_filter, nullptr);
  EXPECT_EQ(device.align, nullptr);
  EXPECT_EQ(device.config, nullptr);
}

TEST_F(DeviceTest, ViamRSDevice_SetValues_StateUpdated) {
  ViamRSDevice<> device;

  device.serial_number = "test123";
  device.started = true;
  device.device = mock_device_;

  EXPECT_EQ(device.serial_number, "test123");
  EXPECT_TRUE(device.started);
  EXPECT_EQ(device.device, mock_device_);
}

// Test destroyDevice function
TEST_F(DeviceTest, DestroyDevice_ValidDevice_ReturnsTrue) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Create a test device
  auto device = std::make_shared<boost::synchronized_value<ViamRSDevice<>>>();
  {
    auto dev_guard = device->synchronize();
    dev_guard->serial_number = "test123";
    dev_guard->started = false;
    dev_guard->pipe = std::make_shared<rs2::pipeline>();
    dev_guard->device = std::make_shared<rs2::device>();
    dev_guard->config = std::make_shared<rs2::config>();
    dev_guard->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    dev_guard->point_cloud_filter = std::make_shared<PointCloudFilter>();
  }

  // Execute
  bool result = destroyDevice(device, logger);

  // Verify function result
  EXPECT_TRUE(result);
  EXPECT_EQ(device, nullptr);

  // Verify logs - device not started, so should NOT have "stopping pipe" log
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log destruction process";

  // Verify no errors
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0) << "Should not log errors for valid device";

  // Verify expected INFO logs (destroying, clearing resources, destroyed)
  // Should NOT contain "stopping pipe" since started=false
  bool found_destroying = false;
  bool found_stopping = false;
  bool found_destroyed = false;

  for (const auto& log : all_logs) {
    if (log.message.find("destroying") != std::string::npos &&
        log.message.find("test123") != std::string::npos) {
      found_destroying = true;
    }
    if (log.message.find("stopping pipe") != std::string::npos) {
      found_stopping = true;
    }
    if (log.message.find("device destroyed") != std::string::npos) {
      found_destroyed = true;
    }
  }

  EXPECT_TRUE(found_destroying) << "Should log 'destroying device'";
  EXPECT_FALSE(found_stopping) << "Should NOT log 'stopping pipe' when device not started";
  EXPECT_TRUE(found_destroyed) << "Should log 'device destroyed'";
}

TEST_F(DeviceTest, DestroyDevice_NullDevice_ReturnsFalse) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  std::shared_ptr<boost::synchronized_value<ViamRSDevice<>>> device = nullptr;

  // Execute
  bool result = destroyDevice(device, logger);

  // Verify function result
  EXPECT_FALSE(result);

  // Verify error log for null device
  auto error_logs = log_capture.get_error_logs();
  ASSERT_EQ(error_logs.size(), 1) << "Should log error for null device";
  EXPECT_THAT(error_logs[0].message,
              ::testing::HasSubstr("trying to destroy an unexistent device"));
}

TEST_F(DeviceTest, DestroyDevice_StartedDevice_StopsAndDestroys) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Create a test device that's started
  auto device = std::make_shared<boost::synchronized_value<
      ViamRSDevice<rs2::device, MockPipeline, MockAlign, MockConfig>>>();
  {
    auto dev_guard = device->synchronize();
    dev_guard->serial_number = "test123";
    dev_guard->started = true; // Device is started
    dev_guard->pipe = std::make_shared<MockPipeline>();
    dev_guard->device = std::make_shared<MockDevice>();
    dev_guard->config = std::make_shared<MockConfig>();
    dev_guard->align = std::make_shared<MockAlign>(RS2_STREAM_COLOR);
    dev_guard->point_cloud_filter = std::make_shared<PointCloudFilter>();
  }

  // Execute
  bool result = destroyDevice(device, logger);

  // Verify function result
  EXPECT_TRUE(result);
  EXPECT_EQ(device, nullptr);

  // Verify logs - device WAS started, so SHOULD have "stopping pipe" log
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log stop and destruction process";

  // Verify no errors
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0) << "Should not log errors for valid started device";

  // Verify expected INFO logs (destroying, stopping pipe, clearing, destroyed)
  bool found_destroying = false;
  bool found_stopping = false;
  bool found_destroyed = false;

  for (const auto& log : all_logs) {
    if (log.message.find("destroying") != std::string::npos &&
        log.message.find("test123") != std::string::npos) {
      found_destroying = true;
    }
    if (log.message.find("stopping pipe") != std::string::npos) {
      found_stopping = true;
    }
    if (log.message.find("device destroyed") != std::string::npos) {
      found_destroyed = true;
    }
  }

  EXPECT_TRUE(found_destroying) << "Should log 'destroying device'";
  EXPECT_TRUE(found_stopping) << "Should log 'stopping pipe' when device was started";
  EXPECT_TRUE(found_destroyed) << "Should log 'device destroyed'";
}

} // namespace test
} // namespace device
} // namespace realsense

GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(
      new realsense::device::test::RealsenseTestEnvironment);
  return RUN_ALL_TESTS();
}
