
import os
import tarfile
from tempfile import TemporaryDirectory

from conan import ConanFile
from conan.api.output import ConanOutput
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy
from conan.internal.deploy import _flatten_directory

class ViamRealsense(ConanFile):
    name = "viam-camera-realsense"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/viam-camera-realsense"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    exports_sources = "CMakeLists.txt", "LICENSE", "src/*", "cmake/*", "meta.json"

    version = "0.0.1"

    def validate(self):
        check_min_cppstd(self, 17)

    def requirements(self):
        self.requires("viam-cpp-sdk/0.20.1")
        self.requires("librealsense/2.56.5")
        self.requires("libjpeg-turbo/[>=2.1.0 <3]")

    def layout(self):
        cmake_layout(self, src_folder=".")

    def generate(self):
        log = ConanOutput(self)
        log.info("Generating build files...")
        tc = CMakeToolchain(self)
        log.info("Setting cache variables...")
        tc.cache_variables["VIAM_REALSENSE_ENABLE_TESTS"] = False
        tc.generate()
        log.info("Generating CMake dependencies...")

        CMakeDeps(self).generate()
        log.info("Build files generated.")

    def build(self):
        log = ConanOutput(self)
        log.info("Building the module...")
        cmake = CMake(self)
        log.info("Configuring the module...")
        cmake.configure()
        log.info("Building the module...")
        cmake.build()
        log.info("Module built.")

    def package(self):
        CMake(self).install()

        # Use CPack to build the module.tar.gz and manually copy it to the package folder
        log = ConanOutput(self)
        log.info("Creating package with CPack...")
        CMake(self).build(target='package')
        log.info("Copying package with CPack...")
        copy(self, pattern="module.tar.gz", src=self.build_folder, dst=self.package_folder)
        log.info("Package created.")

    def deploy(self):
        # For editable packages, package_folder might equal deploy_folder, so copy from build_folder
        # In CI/CD, build_folder might be None, so check it exists first
        log = ConanOutput(self)
        log.info("Creating package with CPack...")
        src = self.package_folder
        log.info(f"Package folder: {self.package_folder}")
        log.info(f"Build folder: {self.build_folder}")
        if self.build_folder and os.path.exists(os.path.join(self.build_folder, "module.tar.gz")):
            src = self.build_folder
        log.info(f"Source folder for deploy: {src}")
        log.info("Copying package with CPack...")
        if src != self.deploy_folder:
            log.info("Copying package with CPack from {} to {}".format(src, self.deploy_folder))
            copy(self, pattern="module.tar.gz", src=src, dst=self.deploy_folder)

        log.info("Package copied.")
