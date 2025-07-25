#include <iostream>
#include <thread>
#include <map>
#include <librealsense2/rs.hpp>

struct my_device {
    ~my_device() {
        std::cout << "deleting device\n";
    }
    std::string serial_number;
    std::shared_ptr<rs2::device> device;
    std::shared_ptr<rs2::pipeline> pipe;
    std::shared_ptr<rs2::pointcloud> pointCloudFilter;
    std::shared_ptr<rs2::align> align;
    // OBCameraParam param;
};
void printDeviceInfo(rs2::device const& dev) {
    std::cout << "DeviceInfo:\n"
              << "  Name:                           " << dev.get_info(RS2_CAMERA_INFO_NAME) << "\n"
              << "  Serial Number:                  " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\n"
              << "  Firmware Version:               " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n"
              << "  Recommended Firmware Version:   " << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION) << "\n"
              << "  ASIC Serial Number:             " << dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << "\n"
              << "  USB Type Descriptor:            " << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << "\n";
}


void startStream(const std::string& serial_number, std::shared_ptr<rs2::device> dev, rs2::pipeline& pipeline) {
    std::cout << "Starting stream for device with serial number: " << serial_number << std::endl;
    // Here you would typically start the stream on the device
    // For example:
    // dev->start();
    rs2::config cfg;
    cfg.enable_device(serial_number);
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    cfg.enable_stream(RS2_STREAM_DEPTH);

    auto profiles = pipeline.start(cfg, [serial_number](rs2::frame const& frame) {
        std::cout << "Received color frame from device: " << serial_number << std::endl;
        // Process the color frame here
    });

    // Collect the enabled streams names
    std::map<int, std::string> stream_names;
    for (auto p : profiles.get_streams())
        stream_names[p.unique_id()] = p.stream_name();
}


int main() {
    std::cout << "starting realsense program" << std::endl;

    rs2::context rs2_ctx;
    rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    rs2::pipeline pipeline(rs2_ctx);


    rs2_ctx.set_devices_changed_callback([&pipeline](rs2::event_information& info) {
        try {
            auto deviceList = info.get_new_devices();

            for (const auto& dev : deviceList) {
                std::cout << "Device Added: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
                printDeviceInfo(dev);
                startStream(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), std::make_shared<rs2::device>(dev), pipeline);
            }
        } catch (rs2::error& e) {
            std::cerr << "Error in devices_changed_callback: " << e.what() << std::endl;
        }
    });


    auto deviceList = rs2_ctx.query_devices();
    for(auto const& dev : deviceList) {
        std::cout << "Device Found: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        printDeviceInfo(dev);
        startStream(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), std::make_shared<rs2::device>(dev), pipeline);
    }
    std::cout << "Device list queried, waiting for device changes..." << std::endl;


    std::cout << "waiting for key press\n";
    std::cin.get();
    std::cout << "stopping orbbec program" << std::endl;

}
