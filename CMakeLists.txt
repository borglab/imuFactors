cmake_minimum_required(VERSION 3.5)
project(ImuFactors LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Enable testing framework
include(CTest)

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)

# Automatically build everything in eval/
file(GLOB eval_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/eval/*.cpp")

foreach(eval_src ${eval_sources})
    get_filename_component(eval_name ${eval_src} NAME_WE)  # filename without extension

    add_executable(${eval_name} ${eval_src})
    target_link_libraries(${eval_name} GTSAM::gtsam CppUnitLite)

    # Register with CTest
    add_test(NAME ${eval_name} COMMAND ${eval_name})
endforeach()
