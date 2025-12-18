
# For Linux: In order to be able to retrieve libudev, PKG_CONFIG_PATH needs to be correctly set 
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	ARCH := $(shell uname -m)
	ifeq ($(ARCH),x86_64)
		export PKG_CONFIG_PATH := /usr/lib/x86_64-linux-gnu/pkgconfig
	else ifeq ($(ARCH),aarch64)
		export PKG_CONFIG_PATH := /usr/lib/aarch64-linux-gnu/pkgconfig
	else ifeq ($(ARCH),arm64)
		export PKG_CONFIG_PATH := /usr/lib/aarch64-linux-gnu/pkgconfig
	endif
endif

OUTPUT_NAME = viam-camera-realsense
BIN := build-conan/build/RelWithDebInfo/viam-camera-realsense

.PHONY: build setup test clean lint conan-pkg

default: module.tar.gz

build: $(BIN)

build/build.ninja: build CMakeLists.txt
	cd build && cmake -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo ..

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
	./bin/lint.sh


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
