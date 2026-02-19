OUTPUT_NAME = viam-camera-realsense
BIN := build-conan/build/RelWithDebInfo/viam-camera-realsense

# OS Detection
UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

# Set architecture-specific paths based on OS
ifeq ($(UNAME_S),Linux)
    export DEFAULT_PKG_CONFIG_PATH := /usr/lib/$(UNAME_M)-linux-gnu/pkgconfig:/usr/share/pkgconfig
    NPROC := $(shell nproc)
else ifeq ($(UNAME_S),Darwin)
    # macOS Homebrew paths
    ifeq ($(UNAME_M),arm64)
        export DEFAULT_PKG_CONFIG_PATH := /opt/homebrew/lib/pkgconfig:/usr/local/lib/pkgconfig
    else
        export DEFAULT_PKG_CONFIG_PATH := /usr/local/lib/pkgconfig
    endif
    NPROC := $(shell sysctl -n hw.ncpu)
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

.PHONY: build setup test clean lint conan-pkg conan-install-test build-native test-native

default: module.tar.gz

conan-install-test: setup
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install . \
	-o "&:with_tests=True" \
	--output-folder=build-conan \
	--build=missing \
	$(CONAN_FLAGS)

# Native CMake build using Conan-provided dependencies
# Faster than using conan build because it uses native CMake directly
# Depends on conan-install-test to generate the toolchain
build-native: conan-install-test
	mkdir -p build-native && cd build-native && \
	cmake .. -DVIAM_REALSENSE_ENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_TOOLCHAIN_FILE=$(CURDIR)/build-conan/build/Release/generators/conan_toolchain.cmake \
	-DCMAKE_PREFIX_PATH=$(CURDIR)/build-conan/build/Release/generators && \
	make -j$(NPROC)

test-native: build-native
	cd build-native && ctest --output-on-failure

# Alias for test-native (main test target)
test: test-native

clean:
	rm -rf build-conan build-native module.tar.gz venv

setup:
	bin/setup.sh

# Both the commands below need to source/activate the venv in the same line as the
# conan call because every line of a Makefile runs in a subshell
conan-pkg: setup
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan create . \
	-o "&:with_tests=False" \
	$(CONAN_FLAGS) \
	--build=missing

module.tar.gz: conan-pkg meta.json
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install --requires=viam-camera-realsense/0.0.1 \
	$(CONAN_FLAGS) \
	--deployer-package "&" \
	--envs-generation false

lint:
	./bin/lint.sh
