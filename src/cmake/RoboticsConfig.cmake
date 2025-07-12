# ==============================================================================
# Robotics Controller Project - Simplified Configuration
# ==============================================================================

# Global project settings
set(ROBOTICS_CXX_STANDARD 17)

# Build configuration defaults
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Common compiler flags
set(ROBOTICS_COMMON_COMPILE_FLAGS
    -Wall
    -Wextra
    -Wpedantic
    -Werror=return-type
)

set(ROBOTICS_RELEASE_COMPILE_FLAGS
    -O3
    -DNDEBUG
)

set(ROBOTICS_DEBUG_COMPILE_FLAGS
    -O0
    -g
    -DDEBUG
)

# Simplified function to configure a robotics target
function(configure_robotics_target target_name)
    set_target_properties(${target_name} PROPERTIES
        CXX_STANDARD ${ROBOTICS_CXX_STANDARD}
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    target_compile_options(${target_name} PRIVATE ${ROBOTICS_COMMON_COMPILE_FLAGS})

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${target_name} PRIVATE ${ROBOTICS_DEBUG_COMPILE_FLAGS})
    else()
        target_compile_options(${target_name} PRIVATE ${ROBOTICS_RELEASE_COMPILE_FLAGS})
    endif()
endfunction()

# Installation directories
set(ROBOTICS_INSTALL_BINDIR "bin")
set(ROBOTICS_INSTALL_LIBDIR "lib")
set(ROBOTICS_INSTALL_INCLUDEDIR "include/robotics-controller")
set(ROBOTICS_INSTALL_DOCDIR "share/doc/robotics-controller")
set(ROBOTICS_INSTALL_CONFIGDIR "share/robotics-controller")
