# ==============================================================================
# Web Interface Installation
# ==============================================================================

# Install web interface files
install(FILES
    index.html
    script.js
    styles.css
    README.md
    DESTINATION share/robotics-controller/www
)

message(STATUS "Web interface configured for installation")
