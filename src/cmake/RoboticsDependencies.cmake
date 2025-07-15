# ==============================================================================
# Robotics Controller Project - Simplified Dependencies
# ==============================================================================

include(FetchContent)

# Configure common dependencies
function(robotics_setup_dependencies)
    # Always required dependencies
    find_package(Threads REQUIRED)

    # JSON library - always fetch from source for consistency
    FetchContent_Declare(nlohmann_json
        URL "https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz"
    )
    FetchContent_MakeAvailable(nlohmann_json)

    # GPIOD library (system only - Yocto should provide this)
    find_path(GPIOD_INCLUDE_DIR gpiod.h)
    find_library(GPIOD_LIBRARY gpiod)

    if(GPIOD_INCLUDE_DIR AND GPIOD_LIBRARY)
        add_library(gpiod UNKNOWN IMPORTED)
        set_target_properties(gpiod PROPERTIES
            IMPORTED_LOCATION ${GPIOD_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${GPIOD_INCLUDE_DIR}
        )
        target_compile_definitions(gpiod INTERFACE HAVE_GPIOD=1)
        message(STATUS "Found GPIOD: ${GPIOD_LIBRARY}")
        set(GPIOD_FOUND TRUE PARENT_SCOPE)
    else()
        message(WARNING "GPIOD not found - creating dummy target")
        add_library(gpiod INTERFACE)
        target_compile_definitions(gpiod INTERFACE NO_GPIOD)
        set(GPIOD_FOUND FALSE PARENT_SCOPE)
    endif()

    # OpenCV library (required for vision processing)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

    # Create imported target for OpenCV
    add_library(opencv INTERFACE IMPORTED)
    target_link_libraries(opencv INTERFACE ${OpenCV_LIBS})
    target_include_directories(opencv INTERFACE ${OpenCV_INCLUDE_DIRS})
    message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
    set(OPENCV_FOUND TRUE PARENT_SCOPE)

    message(STATUS "Dependencies setup complete")
endfunction()
