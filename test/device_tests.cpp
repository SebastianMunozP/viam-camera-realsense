#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <viam/sdk/common/instance.hpp>

#include "device.hpp"
#include "realsense.hpp"

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
namespace test {

// Mock classes for testing
class MockDevice {
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
                         std::vector<std::string>{"color", "depth"}, // sensors
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

  // This test mainly ensures no exceptions are thrown
  EXPECT_NO_THROW(printDeviceInfo(*mock_device_));
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
