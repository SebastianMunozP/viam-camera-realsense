#include <iostream>
#include <thread>
#include <librealsense2/rs.hpp>


int main(int argc, char * argv[]) {
    std::cout << "Starting RealSense Program..." << std::endl;

    rs2::context rs2_ctx;

        auto devices = rs2_ctx.query_devices();

    if (devices.size() == 0) {
        throw std::runtime_error("no devices connected; please connect an Intel RealSense device");
    }

    for (auto const& dev_info_rs2 : devices) {
        std::string current_serial_number;
        current_serial_number = dev_info_rs2.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "found available RealSense device with serial number: "
                            << current_serial_number << std::endl;
    }

    rs2::pipeline p( rs2_ctx );

    p.start();

        while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        auto width = depth.get_width();
        auto height = depth.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
    }

    return EXIT_SUCCESS;
}