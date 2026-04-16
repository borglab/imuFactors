# Local repository tests only.
# Naming convention: tests/testXXX.cpp -> target testXXX -> target testXXX.run

set(IMUFACTORS_TEST_WORKDIR ${CMAKE_BINARY_DIR})

function(imufactors_add_local_test test_name source_file)
    add_executable(${test_name} ${source_file})

    target_link_libraries(${test_name}
        PRIVATE
            imuFactorsEval
            ${IMUFACTORS_GTSAM_TARGET}
            ${IMUFACTORS_CPPUNIT_TARGET}
    )

    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES
        WORKING_DIRECTORY ${IMUFACTORS_TEST_WORKDIR}
    )

    add_custom_target(${test_name}.run
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure -R "^${test_name}$"
        DEPENDS ${test_name}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endfunction()

imufactors_add_local_test(testImuNEES testImuNEES.cpp)
imufactors_add_local_test(testDatasetSanity testDatasetSanity.cpp)
