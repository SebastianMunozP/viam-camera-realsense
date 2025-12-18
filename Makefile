# Package with local librealsense (binary, dylibs, headers, CMake config)
MODULE_LOCAL_TARBALL = module-localrealsense.tar.gz
MODULE_LOCAL_STAGE = module-localrealsense-stage
MODULE_BIN = build-conan/build/RelWithDebInfo/viam-camera-realsense
LRS_LIB_DIR = /usr/local/lib
LRS_INCLUDE_DIR = /usr/local/include/librealsense2
LRS_CMAKE_DIR = /usr/local/lib/cmake/realsense2

.PHONY: module-localrealsense.tar.gz
module-localrealsense.tar.gz: force-rebuild-module bin/run_module_with_sudo.sh
	rm -rf $(MODULE_LOCAL_STAGE) $(MODULE_LOCAL_TARBALL)
	mkdir -p $(MODULE_LOCAL_STAGE)/bin
	mkdir -p $(MODULE_LOCAL_STAGE)/lib
	mkdir -p $(MODULE_LOCAL_STAGE)/include
	mkdir -p $(MODULE_LOCAL_STAGE)/cmake

# Find the latest built viam-camera-realsense binary from the Conan package directory
	CONAN_BIN_PATH=$$(find $$HOME/.conan2/p/b/viam-*/p/viam-camera-realsense -type f -perm +111 -name 'viam-camera-realsense' 2>/dev/null | sort -r | head -n1); \
	cp $$CONAN_BIN_PATH $(MODULE_LOCAL_STAGE)/bin/
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
	rm -rf build-conan/build/RelWithDebInfo
	conan remove 'viam-camera-realsense/*' --confirm || true
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan create . \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--build=missing
OUTPUT_NAME = viam-camera-realsense
BIN := build-conan/build/RelWithDebInfo/viam-camera-realsense

.PHONY: build setup test clean lint conan-pkg

default: module.tar.gz

build: $(BIN)

build/build.ninja: build CMakeLists.txt
	cd build && cmake -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo ..

$(BIN): conanfile.py src/* bin/* test/*
	$(MAKE) module.tar.gz

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
	conan create . \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--build=missing

module.tar.gz: conan-pkg meta.json
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install --requires=viam-camera-realsense/0.0.1 \
	-o:a "viam-cpp-sdk/*:shared=False" \
	-s:a build_type=Release \
	-s:a compiler.cppstd=17 \
	--deployer-package "&" \
	--envs-generation false

lint:
	./bin/run-clang-format.sh

CLI_BUILD_DIR := $(abspath build-cli)
CLI_TOOLCHAIN := $(CLI_BUILD_DIR)/conan_toolchain.cmake

viam-camera-realsense-cli:
	rm -rf $(CLI_BUILD_DIR) && \
	conan remove 'librealsense/*' --confirm && \
	conan install ./src/cli -of=$(CLI_BUILD_DIR) -b missing -s build_type=Debug -o 'librealsense/*:BUILD_EASYLOGGINGPP=True' && \
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
