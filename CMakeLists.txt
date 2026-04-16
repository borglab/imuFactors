cmake_minimum_required(VERSION 3.10)
project(ImuFactors LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable testing framework
include(CTest)

# Find GTSAM (allow local hint to gtsam-private build)
find_package(GTSAM REQUIRED HINTS "$ENV{HOME}/git/gtsam-private/build")

if(TARGET GTSAM::gtsam)
    set(IMUFACTORS_GTSAM_TARGET GTSAM::gtsam)
elseif(TARGET gtsam)
    set(IMUFACTORS_GTSAM_TARGET gtsam)
else()
    message(FATAL_ERROR "Could not find GTSAM CMake target (expected GTSAM::gtsam or gtsam).")
endif()

if(TARGET CppUnitLite::CppUnitLite)
    set(IMUFACTORS_CPPUNIT_TARGET CppUnitLite::CppUnitLite)
elseif(TARGET CppUnitLite)
    set(IMUFACTORS_CPPUNIT_TARGET CppUnitLite)
else()
    message(FATAL_ERROR "Could not find CppUnitLite CMake target.")
endif()

# Shared evaluation code (no main)
add_library(imuFactorsEval
    eval/Dataset.cpp
    eval/NEESEvaluator.cpp
    eval/EKFNEESEvaluator.cpp
    eval/TrajectoryValidator.cpp
)

target_include_directories(imuFactorsEval PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/eval
)

target_link_libraries(imuFactorsEval PUBLIC
    ${IMUFACTORS_GTSAM_TARGET}
)

# CLI entry points
add_executable(evalGal3NavStateImuEKFNEES eval/evalGal3NavStateImuEKFNEES.cpp)
add_executable(evalNoiseCalibration eval/evalNoiseCalibration.cpp)
add_executable(evalExportTrajectories eval/evalExportTrajectories.cpp)

# Link dependencies
foreach(app_target
    evalGal3NavStateImuEKFNEES
    evalNoiseCalibration
    evalExportTrajectories)

    target_link_libraries(${app_target}
        imuFactorsEval
        ${IMUFACTORS_GTSAM_TARGET}
    )
endforeach()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
