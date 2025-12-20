OUTPUT_NAME = viam-camera-realsense
BIN := build-conan/build/RelWithDebInfo/viam-camera-realsense
VERSION = 0.0.3

# OS Detection
UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

# Set architecture-specific paths based on OS
ifeq ($(UNAME_S),Linux)
    export DEFAULT_PKG_CONFIG_PATH := /usr/lib/$(UNAME_M)-linux-gnu/pkgconfig:/usr/share/pkgconfig
else ifeq ($(UNAME_S),Darwin)
    # macOS Homebrew paths
    ifeq ($(UNAME_M),arm64)
        export DEFAULT_PKG_CONFIG_PATH := /opt/homebrew/lib/pkgconfig:/usr/local/lib/pkgconfig
    else
        export DEFAULT_PKG_CONFIG_PATH := /usr/local/lib/pkgconfig
    endif
endif

# Export for sub-processes (like bin/build.sh), does not pollute the parent shell
# Only add the colon if PKG_CONFIG_PATH is already set
ifeq ($(PKG_CONFIG_PATH),)
    export PKG_CONFIG_PATH := $(DEFAULT_PKG_CONFIG_PATH)
else
    export PKG_CONFIG_PATH := $(PKG_CONFIG_PATH):$(DEFAULT_PKG_CONFIG_PATH)
endif
 
# Common Conan settings to ensure binary cache hits across all build flows
export CONAN_FLAGS := -s:a build_type=Release -s:a compiler.cppstd=17

.PHONY: build setup test clean lint conan-pkg conan-build-test conan-install-test build-native test-native

default: module.tar.gz

conan-build-test:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan build . \
	-o "&:with_tests=True" \
	--output-folder=build-conan \
	--build=none \
	$(CONAN_FLAGS) \
	-s:a "&:build_type=RelWithDebInfo"

conan-install-test:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install . \
	-o "&:with_tests=True" \
	--build=missing \
	$(CONAN_FLAGS) \
	-s:a "&:build_type=RelWithDebInfo"

test: conan-install-test conan-build-test
	cd build-conan/build/RelWithDebInfo && . ./generators/conanrun.sh && ctest --output-on-failure

# Native build targets for CI environments with pre-installed dependencies
build-native:
	mkdir -p build-native && cd build-native && \
	cmake .. -DVIAM_REALSENSE_ENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
	make -j$(shell nproc)

test-native: build-native
	cd build-native && ctest --output-on-failure

clean:
	rm -rf build-conan build-native module.tar.gz venv

setup:
	bin/setup.sh

# Both the commands below need to source/activate the venv in the same line as the
# conan call because every line of a Makefile runs in a subshell
conan-pkg:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan create . \
	-o "&:with_tests=False" \
	$(CONAN_FLAGS) \
	--build=missing

module.tar.gz: conan-pkg meta.json
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install --requires=viam-camera-realsense/$(VERSION) \
	$(CONAN_FLAGS) \
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

