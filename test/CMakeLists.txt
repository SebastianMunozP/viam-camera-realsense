# CMakeLists.txt for tests

find_package(GTest REQUIRED)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)


include_directories(${CMAKE_SOURCE_DIR}/src/module)

# Test executables - one per module
add_executable(device_tests device_tests.cpp)
add_executable(encoding_tests encoding_tests.cpp)
add_executable(realsense_tests realsense_tests.cpp)
add_executable(time_tests time_tests.cpp)
add_executable(discovery_tests discovery_tests.cpp)

# Link libraries for each test
target_link_libraries(device_tests
    viamrealsense
    gtest
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

target_link_libraries(encoding_tests
    viamrealsense
    gtest 
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

target_link_libraries(realsense_tests
    viamrealsense
    gtest 
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

target_link_libraries(time_tests
    viamrealsense
    gtest
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

target_link_libraries(discovery_tests
    viamrealsense
    gtest
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

# Register tests
add_test(NAME DeviceTests COMMAND device_tests)
add_test(NAME EncodingTests COMMAND encoding_tests)
add_test(NAME RealsenseTests COMMAND realsense_tests)
add_test(NAME TimeTests COMMAND time_tests)
add_test(NAME DiscoveryTests COMMAND discovery_tests)
