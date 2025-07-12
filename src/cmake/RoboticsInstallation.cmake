# ==============================================================================
# Robotics Controller Project - Simplified Installation
# ==============================================================================

include(GNUInstallDirs)

# Function to setup installation configuration
function(robotics_setup_installation)
    # Set installation prefix if not set
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "/opt/robotics-controller" CACHE PATH "Default install prefix" FORCE)
    endif()
    message(STATUS "Installation configured for: ${CMAKE_INSTALL_PREFIX}")
endfunction()

# Simplified function to install documentation
function(robotics_install_docs)
    cmake_parse_arguments(DOCS
        ""
        ""
        "FILES"
        ${ARGN}
    )

    if(DOCS_FILES)
        install(FILES ${DOCS_FILES}
            DESTINATION ${ROBOTICS_INSTALL_DOCDIR}
        )
    endif()
endfunction()

# Simplified installation summary
function(robotics_installation_summary)
    message(STATUS "Installation summary:")
    message(STATUS "  Prefix: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "  Binaries: ${CMAKE_INSTALL_PREFIX}/${ROBOTICS_INSTALL_BINDIR}")
    message(STATUS "  Libraries: ${CMAKE_INSTALL_PREFIX}/${ROBOTICS_INSTALL_LIBDIR}")
    message(STATUS "  Documentation: ${CMAKE_INSTALL_PREFIX}/${ROBOTICS_INSTALL_DOCDIR}")
endfunction()
