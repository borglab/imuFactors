cmake_minimum_required(VERSION 3.10)
project(imuFactors)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Find required packages
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
# Get CppUnitLite path from GTSAM installation

set(CPPLITE_LIB_PATH ${GTSAM_DIR}/../../../lib/libCppUnitLite.a)
# Include directories
include_directories(
    /home/scottiyio/gtsam-1
    ${GTSAM_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/eval
)
set(NEES_EVALUATOR_SOURCES
    eval/NEESEvaluator.cpp
    eval/Dataset.cpp
    eval/TrajectoryValidator.cpp
    /home/scottiyio/gtsam-1/gtsam/navigation/NavStateImuEKF.cpp
    /home/scottiyio/gtsam-1/gtsam/navigation/Gal3ImuEKF.cpp
)
add_library(nees_evaluator SHARED ${NEES_EVALUATOR_SOURCES})
target_link_libraries(nees_evaluator
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)
set(EKF_NEES_EVALUATOR_SOURCES
    eval/EKFNEESEvaluator.cpp
)
add_library(ekf_nees_evaluator SHARED ${EKF_NEES_EVALUATOR_SOURCES})
target_link_libraries(ekf_nees_evaluator
    nees_evaluator
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)
set(VALIDATION_REPORT_SOURCES
    eval/ValidationReport.cpp
)
add_library(validation_report SHARED ${VALIDATION_REPORT_SOURCES})
target_link_libraries(validation_report
    nees_evaluator
)
# Test NEES evaluation
add_executable(testNEESEvaluation eval/tests/testNEESEvaluation.cpp)
target_link_libraries(testNEESEvaluation
    nees_evaluator
    ekf_nees_evaluator
    ${CPPLITE_LIB_PATH}
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)
add_executable(evalIMUFactor eval/evalImuFactor.cpp)
target_link_libraries(evalIMUFactor
    nees_evaluator
    ekf_nees_evaluator
    ${CPPLITE_LIB_PATH}
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)


add_executable(test_covariance_validation eval/test_covariance_validation.cpp)
target_link_libraries(test_covariance_validation
    nees_evaluator
    ${CPPLITE_LIB_PATH}
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)
add_executable(test_jacobian_validation eval/test_jacobian_validation.cpp)
target_link_libraries(test_jacobian_validation
    nees_evaluator
    ${CPPLITE_LIB_PATH}
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)
add_executable(test_theoretical_validation eval/test_theoretical_validation.cpp)
target_link_libraries(test_theoretical_validation
    nees_evaluator
    ${CPPLITE_LIB_PATH}
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)
# Main evaluation executable (with ValidationReport)
add_executable(evalGal3NavStateImuEKFNEES eval/evalGal3NavStateImuEKFNEES.cpp)
target_link_libraries(evalGal3NavStateImuEKFNEES
    ekf_nees_evaluator
    validation_report
    gtsam
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)
# Install libraries
install(TARGETS nees_evaluator ekf_nees_evaluator validation_report
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
# Install headers
install(FILES
    eval/NEESEvaluator.h
    eval/Dataset.h
    eval/TrajectoryValidator.h
    eval/EKFNEESEvaluator.h
    eval/ValidationReport.h
    DESTINATION include/imuFactors
)
# Install executables
install(TARGETS 
    testNEESEvaluation 
    evalGal3NavStateImuEKFNEES
    test_covariance_validation
    test_jacobian_validation
    test_theoretical_validation
    RUNTIME DESTINATION bin
)
# Enable testing
enable_testing()
# Add tests
add_test(NAME NEESEvaluation COMMAND testNEESEvaluation)
add_test(NAME CovarianceValidation COMMAND test_covariance_validation)
add_test(NAME JacobianValidation COMMAND test_jacobian_validation)
add_test(NAME TheoreticalValidation COMMAND test_theoretical_validation)