# ==============================================================================
# Robotics Controller Project - Simplified Testing
# ==============================================================================

# Function to setup testing environment
function(robotics_setup_testing)
    if(BUILD_TESTS)
        enable_testing()
        include(CTest)
        message(STATUS "Testing framework enabled")
    endif()
endfunction()

# Simplified function to add a component test
function(robotics_add_test test_name)
    cmake_parse_arguments(TEST
        ""
        ""
        "SOURCES;DEPENDENCIES"
        ${ARGN}
    )

    if(NOT BUILD_TESTS)
        return()
    endif()

    if(TEST_SOURCES)
        # Create test executable
        add_executable(${test_name} ${TEST_SOURCES})
        configure_robotics_target(${test_name})

        if(TEST_DEPENDENCIES)
            target_link_libraries(${test_name} PRIVATE ${TEST_DEPENDENCIES})
        endif()

        # Register the test
        add_test(NAME ${test_name} COMMAND ${test_name})
    endif()
endfunction()
