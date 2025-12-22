find_package(GTest REQUIRED)

# On some systems (like older Ubuntu), GMock is a separate package or not 
# exported by GTestConfig.cmake. We check and try to find it explicitly.
if (NOT TARGET GTest::gmock)
    find_package(GMock QUIET)
    if (NOT TARGET GTest::gmock AND TARGET GMock::gmock)
        add_library(GTest::gmock ALIAS GMock::gmock)
    endif()
endif()

# Fallback: if we still don't have gmock, try to link it by name if possible,
# or assume it's part of GTest (though unlikely if targets are missing).
# For now, we prefer targets.

include(GoogleTest)

# Helper library target for linking to the individual test binaries
add_library(viamrealsense_test_common INTERFACE)

target_link_libraries(viamrealsense_test_common
    INTERFACE
        viamrealsense
        GTest::gtest
        GTest::gtest_main
        $<IF:$<TARGET_EXISTS:GTest::gmock>,GTest::gmock,gmock>
        viam-cpp-sdk::viamsdk
)

function(viamrealsense_add_gtest SOURCE_FILE_NAME)
    get_filename_component(TEST_EXECUTABLE_NAME ${SOURCE_FILE_NAME} NAME_WE)

    add_executable(${TEST_EXECUTABLE_NAME} ${SOURCE_FILE_NAME})
    target_link_libraries(${TEST_EXECUTABLE_NAME} PRIVATE viamrealsense_test_common)

    gtest_discover_tests(${TEST_EXECUTABLE_NAME})
endfunction()

viamrealsense_add_gtest(device_tests.cpp)
viamrealsense_add_gtest(encoding_tests.cpp)
viamrealsense_add_gtest(realsense_tests.cpp)
viamrealsense_add_gtest(time_tests.cpp)
viamrealsense_add_gtest(discovery_tests.cpp)
