#include <gtest/gtest.h>

#include "realsense.hpp"
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/components/camera.hpp>

using namespace realsense;
using namespace viam::sdk;

// Mock dependencies and config helpers
class RealsenseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test configuration
        test_serial = "test_device_123456";
        test_name = "test_realsense_camera";
        
        // Set up basic resource config
        auto attributes = ProtoStruct{};
        attributes["serial_number"] = test_serial;
        
        test_config = std::make_unique<ResourceConfig>(
            "rdk:component:camera",     // type
            "",                         // namespace_ (empty)
            test_name,                  // name
            attributes,                 // attributes
            "",                         // converted_attributes (empty)
            Model("viam", "camera", "realsense"), // model
            LinkConfig{},               // frame (empty LinkConfig)
            log_level::info             // log_level
        );

        
        test_deps = Dependencies{};
    }
    
    std::string test_serial;
    std::string test_name;
    std::unique_ptr<ResourceConfig> test_config;
    Dependencies test_deps;
};

// Test RsResourceConfig struct
TEST_F(RealsenseTest, RsResourceConfigConstructorSetsValues) {
    RsResourceConfig config(test_serial, test_name);
    
    EXPECT_EQ(config.serial_number, test_serial);
    EXPECT_EQ(config.resource_name, test_name);
}
