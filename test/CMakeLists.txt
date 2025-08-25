# CMakeLists.txt for tests

find_package(GTest REQUIRED)

add_executable(realsense_tests camera_realsense_test.cpp)

include_directories(${CMAKE_SOURCE_DIR}/src)
target_link_libraries(realsense_tests 
    viamrealsense 
    GTest::GTest 
    viam-cpp-sdk::viamapi
)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)


include_directories(${CMAKE_SOURCE_DIR}/src/module)

# Test executables - one per module
add_executable(device_tests2 device_tests.cpp)
add_executable(encoding_tests2 encoding_tests.cpp)
add_executable(realsense_tests2 realsense_tests.cpp)
add_executable(time_tests2 time_tests.cpp)
add_executable(discovery_tests2 discovery_tests.cpp)

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

target_link_libraries(discovery_tests2
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
add_test(NAME DiscoveryTests COMMAND discovery_tests2)

add_test(NAME CameraTests COMMAND realsense_tests)