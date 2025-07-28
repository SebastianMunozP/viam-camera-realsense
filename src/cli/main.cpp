#include <iostream>
#include <thread>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <fstream>
#include <librealsense2/rs.hpp>

static const double min_distance = 1e-6;

struct PointXYZRGB
{
    float x, y, z;
    unsigned int rgb;
};

bool validPoint(const rs2::vertex &p)
{
    const double min_distance = 1e-6;
    return std::fabs(p.x) >= min_distance || std::fabs(p.y) >= min_distance || std::fabs(p.z) >= min_distance;
}

std::vector<unsigned char> RGBPointsToPCD(std::tuple<rs2::points, rs2::video_frame> data)
{
    const rs2::points &points = std::get<0>(data);
    const rs2::video_frame &color = std::get<1>(data);
    if (!points || !color)
    {
        throw std::runtime_error("Invalid points or color frame");
    }
    const rs2::vertex *vertices = points.get_vertices();
    const rs2::texture_coordinate *tex_coords = points.get_texture_coordinates();
    int num_points = points.size();

    std::vector<PointXYZRGB> pcdPoints;
    const uint8_t *color_data = reinterpret_cast<const uint8_t *>(color.get_data());
    int width = color.get_width();
    int height = color.get_height();

    for (int i = 0; i < num_points; ++i)
    {
        const auto &v = vertices[i];
        if (!validPoint(v))
            continue;

        const auto &tc = tex_coords[i];
        int x = std::min(std::max(int(tc.u * width + 0.5f), 0), width - 1);
        int y = std::min(std::max(int(tc.v * height + 0.5f), 0), height - 1);

        int idx = (y * width + x) * 3;
        unsigned int r = color_data[idx];
        unsigned int g = color_data[idx + 1];
        unsigned int b = color_data[idx + 2];
        unsigned int rgb = (r << 16) | (g << 8) | b;

        PointXYZRGB pt;
        pt.x = v.x;
        pt.y = v.y;
        pt.z = v.z;
        pt.rgb = rgb;
        pcdPoints.push_back(pt);
    }

    std::stringstream header;
    header << "VERSION .7\n"
           << "FIELDS x y z rgb\n"
           << "SIZE 4 4 4 4\n"
           << "TYPE F F F U\n"
           << "COUNT 1 1 1 1\n"
           << "WIDTH " << pcdPoints.size() << "\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << pcdPoints.size() << "\n"
           << "DATA binary\n";
    std::string headerStr = header.str();
    std::vector<unsigned char> pcdBytes;
    pcdBytes.insert(pcdBytes.end(), headerStr.begin(), headerStr.end());
    for (auto &p : pcdPoints)
    {
        unsigned char *x = (unsigned char *)&p.x;
        unsigned char *y = (unsigned char *)&p.y;
        unsigned char *z = (unsigned char *)&p.z;
        unsigned char *rgb = (unsigned char *)&p.rgb;

        pcdBytes.insert(pcdBytes.end(), x, x + 4);
        pcdBytes.insert(pcdBytes.end(), y, y + 4);
        pcdBytes.insert(pcdBytes.end(), z, z + 4);
        pcdBytes.insert(pcdBytes.end(), rgb, rgb + 4);
    }
    return pcdBytes;
}

class PointCloudFilter
{
public:
    PointCloudFilter() : pointcloud(std::make_shared<rs2::pointcloud>()) {}
    std::tuple<rs2::points, rs2::video_frame> process(rs2::frameset frameset)
    {
        auto depth_frame = frameset.get_depth_frame();
        if (!depth_frame)
        {
            throw std::runtime_error("No depth frame in frameset");
        }
        auto color_frame = frameset.get_color_frame();
        if (!color_frame)
        {
            throw std::runtime_error("No color frame in frameset");
        }
        pointcloud->map_to(color_frame);
        auto points = pointcloud->calculate(depth_frame);
        return std::make_tuple(points, color_frame);
    }

private:
    std::shared_ptr<rs2::pointcloud> pointcloud;
};

struct my_device
{
    ~my_device()
    {
        std::cerr << "deleting device\n";
    }
    std::string serial_number;
    std::shared_ptr<rs2::device> device;
    std::shared_ptr<rs2::pipeline> pipe;
    std::shared_ptr<PointCloudFilter> pointCloudFilter;
    std::shared_ptr<rs2::align> align;
};

std::mutex devices_by_serial_mu;
std::unordered_map<std::string, std::unique_ptr<my_device>> devices_by_serial;

std::mutex frame_set_by_serial_mu;
std::unordered_map<std::string, std::shared_ptr<rs2::frameset>> frame_set_by_serial;
void printDeviceInfo(rs2::device const &dev)
{
    std::cout << "DeviceInfo:\n"
              << "  Name:                           " << dev.get_info(RS2_CAMERA_INFO_NAME) << "\n"
              << "  Serial Number:                  " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\n"
              << "  Firmware Version:               " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n"
              << "  Recommended Firmware Version:   " << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION) << "\n"
              << "  ASIC Serial Number:             " << dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << "\n"
              << "  USB Type Descriptor:            " << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << "\n";
}

// check if the given stream profiles are matching in terms of width, height and FPS
// This is used to create a config for software depth-to-color (D2C) alignment
// It checks if the color and depth profiles have the same width, height, and FPS
// If they match, it returns true and prints the matching parameters
// If they do not match, it returns false
// alignment
bool checkIfMatchingColorDepthProfiles(const rs2::video_stream_profile &color,
                                       const rs2::video_stream_profile &depth)
{
    if (color.width() == depth.width() &&
        color.height() == depth.height() &&
        color.fps() == depth.fps())
    {
        std::cout << "using width: " << color.width()
                  << " height: " << color.height()
                  << " fps: " << color.fps() << "\n";
        return true;
    }
    return false;
}

// create a config for software depth-to-color alignment
std::shared_ptr<rs2::config> createSwD2CAlignConfig(std::shared_ptr<rs2::pipeline> pipe, std::shared_ptr<rs2::context> ctx, std::shared_ptr<rs2::device> dev)
{
    auto cfg = std::make_shared<rs2::config>();

    // Query all sensors for the device
    std::vector<rs2::sensor> sensors = dev->query_sensors();
    rs2::sensor color_sensor, depth_sensor;
    for (auto &s : sensors)
    {
        if (s.is<rs2::color_sensor>())
            color_sensor = s;
        if (s.is<rs2::depth_sensor>())
            depth_sensor = s;
    }

    // Get stream profiles
    auto color_profiles = color_sensor.get_stream_profiles();
    auto depth_profiles = depth_sensor.get_stream_profiles();

    // Find matching profiles (same resolution and FPS)
    for (auto &cp : color_profiles)
    {
        auto csp = cp.as<rs2::video_stream_profile>();
        if (csp.format() != RS2_FORMAT_RGB8)
        {
            continue; // Only consider RGB8 format for color
        }
        for (auto &dp : depth_profiles)
        {
            auto dsp = dp.as<rs2::video_stream_profile>();
            if (dsp.format() != RS2_FORMAT_Z16)
            {
                continue; // Only consider Z16 format for depth
            }
            // Check if color and depth profiles match in resolution and FPS
            if (checkIfMatchingColorDepthProfiles(csp, dsp))
            {
                // Enable matching streams in config
                cfg->enable_stream(RS2_STREAM_COLOR, csp.stream_index(), csp.width(), csp.height(), csp.format(), csp.fps());
                cfg->enable_stream(RS2_STREAM_DEPTH, dsp.stream_index(), dsp.width(), dsp.height(), dsp.format(), dsp.fps());
                return cfg;
            }
        }
    }
    // If no match found, return nullptr
    return nullptr;
}

void startStream(const std::string &serial_number, std::shared_ptr<rs2::device> dev, std::shared_ptr<rs2::context> ctx)
{
    auto pipe = std::make_shared<rs2::pipeline>();
    auto pointCloudFilter = std::make_shared<PointCloudFilter>();
    auto align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);

    std::cout << "Starting stream for device with serial number: " << serial_number << std::endl;
    auto cfg = createSwD2CAlignConfig(pipe, ctx, dev);
    if (cfg == nullptr)
    {
        std::cerr << "Failed to create matching color-depth config for device: " << serial_number << std::endl;
        return;
    }
    // Enable the device with the config
    cfg->enable_device(serial_number);
    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu);
        std::unique_ptr<my_device> my_dev = std::make_unique<my_device>();

        my_dev->pipe = pipe;
        my_dev->device = dev;
        my_dev->serial_number = serial_number;
        my_dev->pointCloudFilter = pointCloudFilter;
        my_dev->align = align;

        std::string output_filename = serial_number + "_output.pcd";

        int retries = 5;
        while (retries-- > 0)
        {
            try
            {
                pipe->start(*cfg, [serial_number, align, pointCloudFilter, output_filename](rs2::frame const &frame)
                            {
                            if (frame.is<rs2::frameset>())
                            {
                                // With callbacks, all synchronized stream will arrive in a single frameset
                                auto frameset = std::make_shared<rs2::frameset>(frame.as<rs2::frameset>());
                                if (frameset->size() != 2)
                                {
                                    std::cerr << "got non 2 frame count: " << frameset->size() << std::endl;
                                    return;
                                }
                                auto color_frame = frameset->first_or_default(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
                                if (not color_frame)
                                {
                                    std::cerr << "no color frame" << std::endl;
                                    return;
                                }

                                auto depth_frame = frameset->get_depth_frame();
                                if (not depth_frame)
                                {
                                    std::cerr << "no depth frame" << std::endl;
                                    return;
                                }
                                std::vector<std::uint8_t> data = RGBPointsToPCD(pointCloudFilter->process(align->process(*frameset)));
                                std::ofstream outfile(output_filename, std::ios::out | std::ios::binary);
                                outfile.write((const char *)&data[0], data.size());
                                outfile.close();

                                std::lock_guard<std::mutex> lock(frame_set_by_serial_mu);
                                frame_set_by_serial[serial_number] = frameset;
                            }
                            else
                            {
                                // Stream that bypass synchronization (such as IMU) will produce single frames
                                std::cerr << "got non 2 a frameset: " << frame.get_profile().stream_name() << std::endl;
                                return;
                            } });
                break; // Exit the retry loop if successful
            }
            catch (const rs2::error &e)
            {
                std::string msg = e.what();
                if (msg.find("Device or resource busy") != std::string::npos && retries > 0)
                {
                    std::cerr << "Device busy, retrying in 1s (" << retries << " retries left)...\n";
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
                std::cerr << "RealSense error during pipe->start(): " << msg << " (serial: " << serial_number << ")" << std::endl;
                break;
            }
        }

        devices_by_serial[serial_number] = std::move(my_dev);
    }
    std::cout << "Started streaming from device with serial number: " << serial_number << std::endl;
}

void stopStream(std::unique_ptr<my_device> const &my_dev)
{
    if (!my_dev)
    {
        std::cerr << "Device is null, cannot stop stream." << std::endl;
        return;
    }
    std::cout << "Stopping stream for device: " << my_dev->serial_number << std::endl;
    if (!my_dev->pipe)
    {
        std::cerr << "Pipeline is null, cannot stop stream." << std::endl;
        return;
    }
    try
    {
        my_dev->pipe->stop();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception during pipe->stop(): " << e.what() << std::endl;
    }
    std::cout << "Stopped stream for device: " << my_dev->serial_number << std::endl;
}

void stopStreams()
{
    std::cout << "Stopping all streams" << std::endl;
    for (auto &[key, my_device] : devices_by_serial)
    {
        std::cout << "stop stream " << key << "\n";
        stopStream(my_device);
    }
    std::cout << "All streams stopped" << std::endl;
}

std::vector<std::shared_ptr<rs2::device>> get_removed_devices(rs2::event_information &info)
{
    // This function checks if any devices were removed during the callback
    // Create a snapshot of the current devices_by_serial map
    std::vector<std::shared_ptr<rs2::device>> devices_snapshot;
    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu);
        for (const auto &[serial_number, device] : devices_by_serial)
        {
            devices_snapshot.push_back(device->device);
        }
    }

    // Check for removed devices
    std::vector<std::shared_ptr<rs2::device>> removed_devices;
    for (auto const &device : devices_snapshot)
    {
        // Check if the device was removed
        std::cerr << "Checking if device was removed: " << device->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        if (info.was_removed(*device))
        {
            std::string serial_number = device->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            std::cout << "Device Removed: " << serial_number << std::endl;
            if (devices_by_serial.count(serial_number) > 0)
            {
                // Stop the stream for the removed device
                stopStream(devices_by_serial[serial_number]);
            }
            else
            {
                std::cerr << "Removed device " << serial_number << " not found in devices_by_serial.\n";
            }
            // Add to removed devices list
            removed_devices.push_back(device);
        }
    }
    return removed_devices;
}

int main()
{
    std::cout << "starting realsense program" << std::endl;

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    auto ctx = std::make_shared<rs2::context>();

    ctx->set_devices_changed_callback([ctx](rs2::event_information &info)
                                      {
        try {
            // Handling removed devices, if any
            // the callback api does not provide a list of removed devices, so we need to check the current device list for if one or more of them was removed
            auto removed_devices = get_removed_devices(info);
            for (const auto &dev : removed_devices)
            {
                std::string dev_serial_number = dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                std::cout << "Device Removed: " << dev_serial_number << std::endl;
                if (devices_by_serial.count(dev_serial_number) > 0)
                {
                    // Stop the stream for the removed device
                    stopStream(devices_by_serial[dev_serial_number]);

                    // Remove from devices_by_serial
                    std::lock_guard<std::mutex> lock(devices_by_serial_mu);
                    devices_by_serial.erase(dev_serial_number);
                }
                else
                    std::cerr << "Removed device " << dev_serial_number << " not found in devices_by_serial.\n";
            }

            // Handling added devices, if any
            auto added_devices = info.get_new_devices();
            for (const auto& dev : added_devices) {
                std::cout << "Device Added: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
                printDeviceInfo(dev);
                startStream(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), std::make_shared<rs2::device>(dev), ctx);
            }
        } catch (rs2::error& e) {
            std::cerr << "Error in devices_changed_callback: " << e.what() << std::endl;
        } });

    // This will the initial set of connected devices (i.e. the devices that were connected before the callback was set)
    auto deviceList = ctx->query_devices();
    for (auto const &dev : deviceList)
    {
        if (devices_by_serial.count(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) > 0)
        {
            std::cout << "Device already in devices_by_serial: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
            continue; // Skip if already in devices_by_serial
        }
        std::cout << "Device Added: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        printDeviceInfo(dev);
        startStream(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), std::make_shared<rs2::device>(dev), ctx);
    }
    std::cout << "Device list queried, waiting for device changes..." << std::endl;

    std::cout << "waiting for key press\n";
    std::cin.get();
    std::cout << "stopping orbbec program" << std::endl;

    stopStreams();
    std::lock_guard<std::mutex> lock(devices_by_serial_mu);
    devices_by_serial.clear();
    std::lock_guard<std::mutex> frame_lock(frame_set_by_serial_mu);
    frame_set_by_serial.clear();
    std::cout << "stopped orbbec program" << std::endl;
    return 0;
}
