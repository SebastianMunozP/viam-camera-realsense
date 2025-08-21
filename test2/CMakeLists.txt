# find_package(GTest REQUIRED)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src/module)

# Test executables - one per module
add_executable(device_tests2 device_tests.cpp)
add_executable(encoding_tests2 encoding_tests.cpp)
add_executable(realsense_tests2 realsense_tests.cpp)
add_executable(time_tests2 time_tests.cpp)

# Link libraries for each test
target_link_libraries(device_tests2
    viamrealsense2
    gtest
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

target_link_libraries(encoding_tests2 
    viamrealsense2
    gtest 
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

target_link_libraries(realsense_tests2
    viamrealsense2
    gtest 
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

target_link_libraries(time_tests2
    viamrealsense2
    gtest
    gtest_main  # Provides main() function automatically
    gmock
    viam-cpp-sdk::viamapi
)

# Register tests
add_test(NAME DeviceTests COMMAND device_tests2)
add_test(NAME EncodingTests COMMAND encoding_tests2)
add_test(NAME RealsenseTests COMMAND realsense_tests2)
add_test(NAME TimeTests COMMAND time_tests2)

# Optional: Create a meta-target to build all tests
add_custom_target(all_tests)
add_dependencies(all_tests device_tests2 encoding_tests2 realsense_tests2 time_tests2)
