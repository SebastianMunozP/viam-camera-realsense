#include <gtest/gtest.h>
#include "../src/module/sensors.hpp"

namespace realsense {
namespace sensors {

TEST(SensorsTest, SensorTypeToString) {
    EXPECT_EQ(sensor_type_to_string(SensorType::depth), "depth");
    EXPECT_EQ(sensor_type_to_string(SensorType::color), "color");
    EXPECT_EQ(sensor_type_to_string(SensorType::unknown), "unknown");
}

TEST(SensorsTest, StringToSensorType) {
    EXPECT_EQ(string_to_sensor_type("depth"), SensorType::depth);
    EXPECT_EQ(string_to_sensor_type("DEPTH"), SensorType::depth);
    EXPECT_EQ(string_to_sensor_type("Depth"), SensorType::depth);
    
    EXPECT_EQ(string_to_sensor_type("color"), SensorType::color);
    EXPECT_EQ(string_to_sensor_type("COLOR"), SensorType::color);
    EXPECT_EQ(string_to_sensor_type("Color"), SensorType::color);
    
    EXPECT_EQ(string_to_sensor_type("unknown"), SensorType::unknown);
    EXPECT_EQ(string_to_sensor_type("foo"), SensorType::unknown);
    EXPECT_EQ(string_to_sensor_type(""), SensorType::unknown);
}

// Dummy classes to mock rs2::sensor-like interface for get_sensor_type template
struct MockDepthSensor {
    template <typename T> bool is() const {
        return std::is_same<T, rs2::depth_sensor>::value;
    }
};

struct MockColorSensor {
    template <typename T> bool is() const {
        return std::is_same<T, rs2::color_sensor>::value;
    }
};

struct MockUnknownSensor {
    template <typename T> bool is() const {
        return false;
    }
};

TEST(SensorsTest, GetSensorType) {
    MockDepthSensor depth_sensor;
    EXPECT_EQ(get_sensor_type(depth_sensor), SensorType::depth);
    
    MockColorSensor color_sensor;
    EXPECT_EQ(get_sensor_type(color_sensor), SensorType::color);
    
    MockUnknownSensor unknown_sensor;
    EXPECT_EQ(get_sensor_type(unknown_sensor), SensorType::unknown);
}

} // namespace sensors
} // namespace realsense
