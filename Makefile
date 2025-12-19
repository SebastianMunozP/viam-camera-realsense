# Package with local librealsense (binary, dylibs, headers, CMake config)
MODULE_LOCAL_TARBALL = module-localrealsense.tar.gz
MODULE_LOCAL_STAGE = module-localrealsense-stage
MODULE_BIN = build-conan/build/RelWithDebInfo/viam-camera-realsense
LRS_LIB_DIR = /Users/sebastian.munoz/Repos/librealsense/build/Debug
LRS_INCLUDE_DIR = /Users/sebastian.munoz/Repos/librealsense/include/librealsense2
LRS_CMAKE_DIR = /Users/sebastian.munoz/Repos/librealsense/build

.PHONY: module-localrealsense.tar.gz
module-localrealsense.tar.gz: force-rebuild-module bin/run_module_with_sudo.sh
	rm -rf $(MODULE_LOCAL_STAGE) $(MODULE_LOCAL_TARBALL)
	mkdir -p $(MODULE_LOCAL_STAGE)/bin
	mkdir -p $(MODULE_LOCAL_STAGE)/lib
	mkdir -p $(MODULE_LOCAL_STAGE)/include
	mkdir -p $(MODULE_LOCAL_STAGE)/cmake

# Copy the binary from the local build directory (layout is usually build/Release on Mac)
	cp build/Release/viam-camera-realsense $(MODULE_LOCAL_STAGE)/bin/
	cp bin/run_module_with_sudo.sh $(MODULE_LOCAL_STAGE)/bin/
	chmod +x $(MODULE_LOCAL_STAGE)/bin/run_module_with_sudo.sh
	cp $(LRS_LIB_DIR)/librealsense2*.dylib $(MODULE_LOCAL_STAGE)/lib/ || true
	cp -R $(LRS_INCLUDE_DIR) $(MODULE_LOCAL_STAGE)/include/
	cp -R $(LRS_CMAKE_DIR) $(MODULE_LOCAL_STAGE)/cmake/

	echo '{' > $(MODULE_LOCAL_STAGE)/meta.json
	echo '  "name": "viam-camera-realsense",' >> $(MODULE_LOCAL_STAGE)/meta.json
	echo '  "version": "0.0.1",' >> $(MODULE_LOCAL_STAGE)/meta.json
	echo '  "entrypoint": "bin/run_module_with_sudo.sh"' >> $(MODULE_LOCAL_STAGE)/meta.json
	echo '}' >> $(MODULE_LOCAL_STAGE)/meta.json

	tar czvf $(MODULE_LOCAL_TARBALL) -C $(MODULE_LOCAL_STAGE) .
	echo "Created $(MODULE_LOCAL_TARBALL) with sudo wrapper, module binary, librealsense dylibs, headers, CMake config, and meta.json."

# Always force a rebuild from source before packaging
.PHONY: force-rebuild-module
force-rebuild-module:
	rm -rf build-conan
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install . \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--build=missing; \
	conan build . -s build_type=Release
OUTPUT_NAME = viam-camera-realsense
BIN := build-conan/build/RelWithDebInfo/viam-camera-realsense

# Get the architecture-specific pkgconfig path dynamically
ARCH_PATH = /usr/lib/$(shell uname -m)-linux-gnu/pkgconfig
DEFAULT_PKG_CONFIG_PATH = $(ARCH_PATH):/usr/share/pkgconfig

.PHONY: build setup test clean lint conan-pkg

default: module.tar.gz

build: $(BIN)

$(BIN): conanfile.py src/* bin/* test/*
	bin/build.sh

test: $(BIN)
	cd build-conan/build/RelWithDebInfo && ctest --output-on-failure

clean:
	rm -rf build-conan/build/RelWithDebInfo module.tar.gz

setup:
	bin/setup.sh

# Both the commands below need to source/activate the venv in the same line as the
# conan call because every line of a Makefile runs in a subshell
conan-pkg:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	PKG_CONFIG_PATH=$${PKG_CONFIG_PATH}:$(DEFAULT_PKG_CONFIG_PATH) conan create . \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--build=missing

module.tar.gz: conan-pkg meta.json
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	PKG_CONFIG_PATH=$${PKG_CONFIG_PATH}:$(DEFAULT_PKG_CONFIG_PATH) conan install --requires=viam-camera-realsense/0.0.1 \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--deployer-package "&" \
	--envs-generation false

lint:
	./bin/lint.sh

CLI_BUILD_DIR := $(abspath build-cli)
CLI_TOOLCHAIN := $(CLI_BUILD_DIR)/conan_toolchain.cmake

viam-camera-realsense-cli:
	rm -rf $(CLI_BUILD_DIR) && \
	conan remove 'librealsense/*' --confirm && \
	conan install ./src/cli -of=$(CLI_BUILD_DIR) -b missing -s build_type=Debug && \
	test -f $(CLI_TOOLCHAIN) && \
	cmake -S src/cli -B $(CLI_BUILD_DIR) -G Ninja -DCMAKE_TOOLCHAIN_FILE=$(CLI_TOOLCHAIN) -DCMAKE_BUILD_TYPE=Debug && \
	cmake --build $(CLI_BUILD_DIR) --target viam-camera-realsense-cli

# Docker
BUILD_CMD = docker buildx build --pull $(BUILD_PUSH) --force-rm --no-cache --build-arg MAIN_TAG=$(MAIN_TAG) --build-arg BASE_TAG=$(BUILD_TAG) --platform linux/$(BUILD_TAG) -f $(BUILD_FILE) -t '$(MAIN_TAG):$(BUILD_TAG)' .
BUILD_PUSH = --load
BUILD_FILE = ./etc/Dockerfile.debian.bookworm

docker: docker-build docker-upload

docker-build: docker-arm64

docker-arm64: MAIN_TAG = ghcr.io/viam-modules/viam-camera-realsense
docker-arm64: BUILD_TAG = arm64
docker-arm64:
	$(BUILD_CMD)

docker-upload:
	docker push 'ghcr.io/viam-modules/viam-camera-realsense:arm64'

# CI targets that automatically push, avoid for local test-first-then-push workflows
docker-arm64-ci: MAIN_TAG = ghcr.io/viam-modules/viam-camera-realsense
docker-arm64-ci: BUILD_TAG = arm64
docker-arm64-ci: BUILD_PUSH = --push
docker-arm64-ci:
	$(BUILD_CMD)

docker-amd64-ci: MAIN_TAG = ghcr.io/viam-modules/viam-camera-realsense
docker-amd64-ci: BUILD_TAG = amd64
docker-amd64-ci: BUILD_PUSH = --push
docker-amd64-ci:
	$(BUILD_CMD)
