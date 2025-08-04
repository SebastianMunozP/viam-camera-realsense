#pragma once
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <librealsense2/rs.hpp>

#include <string>
#include <vector>

namespace realsense {

// The native config struct for realsense resources.
struct RsResourceConfig {
    std::string resource_name;
    std::string serial_number;

    explicit RsResourceConfig(std::string const& serial_number, std::string const& resource_name)
        : serial_number(serial_number), resource_name(resource_name) {}
};

void startRealsenseSDK(std::shared_ptr<rs2::context> ctx);
void printDeviceInfo(const std::shared_ptr<rs2::device> info);

class Realsense final : public viam::sdk::Camera, public viam::sdk::Reconfigurable {
   public:
    Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg);
    ~Realsense();
    void reconfigure(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg) override;
    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;
    raw_image get_image(std::string mime_type, const viam::sdk::ProtoStruct& extra) override;
    image_collection get_images() override;
    point_cloud get_point_cloud(std::string mime_type, const viam::sdk::ProtoStruct& extra) override;
    properties get_properties() override;
    std::vector<viam::sdk::GeometryConfig> get_geometries(const viam::sdk::ProtoStruct& extra) override;

    static viam::sdk::GeometryConfig geometry;
    static viam::sdk::Model model;

   private:
    std::unique_ptr<RsResourceConfig> config_;
    std::mutex config_mu_;
    static std::unique_ptr<RsResourceConfig> configure_(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg);
};

}  // namespace realsense

