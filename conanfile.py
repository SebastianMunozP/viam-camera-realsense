import os
import tarfile
import re
import json
from tempfile import TemporaryDirectory

from conan import ConanFile
from conan.api.output import ConanOutput
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, load
from conan.internal.deploy import _flatten_directory

class ViamRealsense(ConanFile):
    name = "viam-camera-realsense"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/viam-camera-realsense"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"
    options = {"with_tests": [True, False]}
    default_options = {
        "with_tests": False,
        "viam-cpp-sdk/*:shared": False
    }

    # We need to vendor librealsense for macOS, so we include it in exports_sources. Also, we need to include the bin directory for the sudo wrapper and build_librealsense_macos.sh script
    exports_sources = "CMakeLists.txt", "LICENSE", "src/*", "cmake/*", "meta.json", "test/*", "vendor/librealsense-install/*", "bin/*"

    version = "0.0.1"

    def set_version(self):
        content = load(self, "CMakeLists.txt")
        self.version = re.search("set\(CMAKE_PROJECT_VERSION (.+)\)", content).group(1).strip()

    def validate(self):
        check_min_cppstd(self, 17)

    def requirements(self):
        # This version is 
        # self.requires("viam-cpp-sdk/0.20.1#548747da23ebc8e126ee099e7ee1bc4a")
        self.requires("viam-cpp-sdk/0.20.1")
        # At the moement we are vendoring librealsense for macOS
        if self.settings.os != "Macos":
            self.requires("librealsense/2.56.5")
        self.requires("libjpeg-turbo/[>=2.1.0 <3]")
        
    def layout(self):
        cmake_layout(self, src_folder=".")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["VIAM_REALSENSE_ENABLE_TESTS"] = self.options.with_tests
        tc.generate()

        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        # Also package the sudo wrapper
        copy(self, "run_module_with_sudo.sh", src=os.path.join(self.export_sources_folder, "bin"), dst=os.path.join(self.package_folder, "bin"))

    def deploy(self):
        with TemporaryDirectory(dir=self.deploy_folder) as tmp_dir:
            self.output.debug(f"Creating temporary directory {tmp_dir}")
            
            # Create bin directory
            os.makedirs(os.path.join(tmp_dir, "bin"), exist_ok=True)

            self.output.info("Deploying necessary files to module.tar.gz")
            
            # Copy the main binary to bin/
            copy(self, "viam-camera-realsense", src=self.package_folder, dst=os.path.join(tmp_dir, "bin"))
            
            # Copy meta.json to root
            copy(self, "meta.json", src=self.package_folder, dst=tmp_dir)
            
            # Copy sudo wrapper from package_folder to bin/
            copy(self, "run_module_with_sudo.sh", src=os.path.join(self.package_folder, "bin"), dst=os.path.join(tmp_dir, "bin"))
                
            # Update meta.json entrypoint to use sudo wrapper
            meta_path = os.path.join(tmp_dir, "meta.json")
            with open(meta_path, "r") as f:
                meta = json.load(f)
            
            meta["entrypoint"] = "bin/run_module_with_sudo.sh"
            
            with open(meta_path, "w") as f:
                json.dump(meta, f, indent=2)

            # Ensure executable permissions for scripts and binaries
            os.chmod(os.path.join(tmp_dir, "bin", "run_module_with_sudo.sh"), 0o755)
            os.chmod(os.path.join(tmp_dir, "bin", "viam-camera-realsense"), 0o755)

            self.output.info("Creating module.tar.gz")
            with tarfile.open(os.path.join(self.deploy_folder, "module.tar.gz"), "w|gz") as tar:
                tar.add(tmp_dir, arcname=".", recursive=True)

                self.output.info("module.tar.gz contents:")
                for mem in tar.getmembers():
                    self.output.info(mem.name)
