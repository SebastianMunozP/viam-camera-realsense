include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)


# Device tests
add_executable(device_tests device_tests.cpp)
target_link_libraries(device_tests PRIVATE viamrealsense GTest::gtest GTest::gtest_main GTest::gmock)
gtest_discover_tests(device_tests DISCOVERY_TIMEOUT 60)

# Encoding tests (needs turbojpeg)
add_executable(encoding_tests encoding_tests.cpp)
target_link_libraries(encoding_tests PRIVATE viamrealsense GTest::gtest GTest::gtest_main GTest::gmock)
gtest_discover_tests(encoding_tests DISCOVERY_TIMEOUT 60)

# Realsense tests
add_executable(realsense_tests realsense_tests.cpp)
target_link_libraries(realsense_tests PRIVATE viamrealsense GTest::gtest GTest::gtest_main GTest::gmock)
gtest_discover_tests(realsense_tests DISCOVERY_TIMEOUT 60)

# Time tests
add_executable(time_tests time_tests.cpp)
target_link_libraries(time_tests PRIVATE viamrealsense GTest::gtest GTest::gtest_main GTest::gmock)
gtest_discover_tests(time_tests DISCOVERY_TIMEOUT 60)

# Discovery tests
add_executable(discovery_tests discovery_tests.cpp)
target_link_libraries(discovery_tests PRIVATE viamrealsense GTest::gtest GTest::gtest_main GTest::gmock)
gtest_discover_tests(discovery_tests DISCOVERY_TIMEOUT 60)
