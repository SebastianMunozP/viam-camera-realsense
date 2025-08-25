#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "module/discovery.hpp"
#include <viam/sdk/common/instance.hpp>
#include <librealsense2/rs.hpp>

viam::sdk::Instance global_instance;

class MockDevice {
    std::string serial_number_;
public:
    MockDevice() = default;
    MockDevice(const std::string& serial) : serial_number_(serial) {}
    bool supports(rs2_camera_info info) const {
        if (info == RS2_CAMERA_INFO_SERIAL_NUMBER) {
            return true;
        }
        return false;
    }
    std::string get_info(rs2_camera_info info) const {
        if (info == RS2_CAMERA_INFO_SERIAL_NUMBER) {
            return serial_number_;
        }
        return "";
    }
};

class IContext {
public:
    virtual ~IContext() = default;
    virtual std::vector<MockDevice> query_devices() const = 0;
};

class MockContext : public IContext {
    std::vector<MockDevice> devices_;
public:
    MockContext() = default;

    MockContext(std::vector<std::string> const& serial_numbers) {
        for (const auto& serial : serial_numbers) {
            devices_.emplace_back(serial);
        }
        ON_CALL(*this, query_devices()).WillByDefault(::testing::Return(devices_));
    }

    MOCK_METHOD(std::vector<MockDevice>, query_devices, (), (const, override));
};

TEST(RealsenseDiscoveryTest, NoDevicesFound) {
    auto mock_ctx = std::make_shared<MockContext>();
    std::vector<MockDevice> empty_list;
    EXPECT_CALL(*mock_ctx, query_devices()).WillOnce(::testing::Return(empty_list));

    realsense::discovery::RealsenseDiscovery<IContext> discovery({}, viam::sdk::ResourceConfig("discovery"), mock_ctx);
    auto configs = discovery.discover_resources({});
    EXPECT_TRUE(configs.empty());
}

TEST(RealsenseDiscoveryTest, DeviceWithSerialFound) {
    auto test_ctx = std::make_shared<MockContext>(std::vector<std::string>{"123456"});

    realsense::discovery::RealsenseDiscovery<IContext> discovery({}, viam::sdk::ResourceConfig("discovery"), test_ctx);
    auto configs = discovery.discover_resources({});
    ASSERT_EQ(configs.size(), 1);
    EXPECT_EQ(configs[0].name(), "realsense-123456");
    EXPECT_EQ(configs[0].attributes().at("serial_number"), "123456");
}

TEST(RealsenseDiscoveryTest, MultipleDevicesFound) {
    auto test_ctx = std::make_shared<MockContext>(std::vector<std::string>{"123456", "789"});

    realsense::discovery::RealsenseDiscovery<IContext> discovery({}, viam::sdk::ResourceConfig("discovery"), test_ctx);
    auto configs = discovery.discover_resources({});
    ASSERT_EQ(configs.size(), 2);
    EXPECT_EQ(configs[0].name(), "realsense-123456");
    EXPECT_EQ(configs[0].attributes().at("serial_number"), "123456");
    EXPECT_EQ(configs[1].name(), "realsense-789");
    EXPECT_EQ(configs[1].attributes().at("serial_number"), "789");
}

