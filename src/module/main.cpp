#include "realsense.hpp"

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/module/service.hpp>

#include <iostream>
#include <memory>

#include <librealsense2/rs.hpp>

namespace vsdk = ::viam::sdk;

std::vector<std::shared_ptr<vsdk::ModelRegistration>>
create_all_model_registrations(
    std::shared_ptr<realsense::RealsenseContext<rs2::context>> ctx) {
  std::vector<std::shared_ptr<vsdk::ModelRegistration>> registrations;

  registrations.push_back(std::make_shared<vsdk::ModelRegistration>(
      vsdk::API::get<vsdk::Camera>(), realsense::Realsense<rs2::context>::model,
      [ctx](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
        return std::make_unique<realsense::Realsense<rs2::context>>(
            std::move(deps), std::move(config), ctx);
      },
      realsense::Realsense<
          realsense::RealsenseContext<rs2::context>>::validate));

  // TODO: Add discovery model registration as well

  return registrations;
}

int serve(int argc, char **argv) try {
  // Every Viam C++ SDK program must have one and only one Instance object
  // which is created before any other C++ SDK objects and stays alive until
  // all Viam C++ SDK objects are destroyed.
  vsdk::Instance inst;

  for (size_t i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--log-level=debug") {
      rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    }
  }

  auto rs_ctx = std::make_shared<rs2::context>();
  auto ctx =
      std::make_shared<realsense::RealsenseContext<rs2::context>>(rs_ctx);

  auto module_service = std::make_shared<vsdk::ModuleService>(
      argc, argv, create_all_model_registrations(ctx));
  module_service->serve();

  return EXIT_SUCCESS;
} catch (const std::exception &ex) {
  std::cerr << "ERROR: A std::exception was thrown from `serve`: " << ex.what()
            << std::endl;
  return EXIT_FAILURE;
} catch (...) {
  std::cerr << "ERROR: An unknown exception was thrown from `serve`"
            << std::endl;
  return EXIT_FAILURE;
}

int main(int argc, char *argv[]) {
  std::cout << "Realsense C++ SDK version: " << RS2_API_VERSION_STR << "\n";
  const std::string usage = "usage: realsense /path/to/unix/socket";
  if (argc < 2) {
    std::cout << "ERROR: insufficient arguments\n";
    std::cout << usage << "\n";
    return EXIT_FAILURE;
  }

  return serve(argc, argv);
};
