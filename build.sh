#!/bin/bash

# ==============================================================================
# Robotics Controller Modular Build Script
# ==============================================================================

set -e  # Exit on any error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR"
PROJECT_ROOT="$(dirname "$SRC_DIR")"

# Default values
BUILD_TYPE="Release"
BUILD_DIR="$SRC_DIR/build"
CLEAN_BUILD=false
BUILD_TESTS=false
ENABLE_DEBUG=false
BUILD_INDIVIDUAL=true
BUILD_VISION=true
VERBOSE=false
JOBS=$(nproc)

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    cat << EOF
Usage: $0 [OPTIONS] [TARGET]

Build the Robotics Controller project with modular CMake structure.

OPTIONS:
    -h, --help              Show this help message
    -c, --clean             Clean build (remove build directory first)
    -d, --debug             Build in Debug mode (default: Release)
    -t, --tests             Build with tests enabled
    -v, --verbose           Verbose build output
    -j, --jobs N            Number of parallel jobs (default: $(nproc))
    --no-individual         Don't build individual component targets
    --no-vision             Disable OpenCV vision support
    --build-dir DIR         Custom build directory (default: src/build)

TARGETS:
    all                     Build everything (default)
    robotics-controller     Build only the main controller
    web-interface           Build only web interface
    tests                   Build and run tests
    clean                   Clean build artifacts
    install                 Install built components

EXAMPLES:
    $0                      # Build everything in Release mode
    $0 --clean --debug      # Clean build in Debug mode
    $0 --tests robotics-controller  # Build controller with tests
    $0 --jobs 4 --verbose   # Build with 4 jobs and verbose output

EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -d|--debug)
            BUILD_TYPE="Debug"
            ENABLE_DEBUG=true
            shift
            ;;
        -t|--tests)
            BUILD_TESTS=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        --no-individual)
            BUILD_INDIVIDUAL=false
            shift
            ;;
        --no-vision)
            BUILD_VISION=false
            shift
            ;;
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --*)
            print_error "Unknown option: $1"
            exit 1
            ;;
        *)
            TARGET="$1"
            shift
            ;;
    esac
done

# Set default target
TARGET=${TARGET:-"all"}

# Check prerequisites
check_prerequisites() {
    print_status "Checking prerequisites..."

    # Check for required tools
    local missing_tools=""
    for tool in cmake make g++; do
        if ! command -v $tool &> /dev/null; then
            missing_tools="$missing_tools $tool"
        fi
    done

    if [ -n "$missing_tools" ]; then
        print_error "Missing required tools:$missing_tools"
        print_status "On Ubuntu/Debian: sudo apt-get install cmake build-essential"
        exit 1
    fi

    # Check for optional dependencies
    if ! pkg-config --exists libgpiod; then
        print_warning "libgpiod not found - GPIO functionality may be limited"
        print_status "Install with: sudo apt-get install libgpiod-dev"
    fi

    print_success "Prerequisites check completed"
}

# Clean build directory
clean_build() {
    if [ "$CLEAN_BUILD" = true ] && [ -d "$BUILD_DIR" ]; then
        print_status "Cleaning build directory: $BUILD_DIR"
        rm -rf "$BUILD_DIR"
    fi
}

# Configure CMake
configure_cmake() {
    print_status "Configuring CMake..."

    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    local cmake_args=(
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
        -DBUILD_TESTS="$BUILD_TESTS"
        -DENABLE_DEBUG_LOGGING="$ENABLE_DEBUG"
        -DBUILD_INDIVIDUAL_TARGETS="$BUILD_INDIVIDUAL"
        -DBUILD_VISION_SUPPORT="$BUILD_VISION"
    )

    if [ "$VERBOSE" = true ]; then
        cmake_args+=(-DCMAKE_VERBOSE_MAKEFILE=ON)
    fi

    cmake "${cmake_args[@]}" "$SRC_DIR"

    print_success "CMake configuration completed"
}

# Build the project
build_project() {
    print_status "Building target: $TARGET"

    cd "$BUILD_DIR"

    local make_args=(-j "$JOBS")
    if [ "$VERBOSE" = true ]; then
        make_args+=(VERBOSE=1)
    fi

    case "$TARGET" in
        all)
            make "${make_args[@]}"
            ;;
        robotics-controller)
            make "${make_args[@]}" robotics-controller-exe
            ;;
        web-interface)
            if command -v npm &> /dev/null; then
                make "${make_args[@]}" web-build
            else
                print_warning "npm not found - skipping web interface build"
            fi
            ;;
        tests)
            if [ "$BUILD_TESTS" != true ]; then
                print_error "Tests not enabled. Use --tests flag."
                exit 1
            fi
            make "${make_args[@]}"
            make test
            ;;
        clean)
            make clean
            ;;
        install)
            make "${make_args[@]}"
            sudo make install
            ;;
        *)
            # Try to build the specific target
            make "${make_args[@]}" "$TARGET" || {
                print_error "Unknown target: $TARGET"
                print_status "Available targets:"
                make help | grep "^\\.\\.\\." | head -20
                exit 1
            }
            ;;
    esac

    print_success "Build completed successfully"
}

# Show build summary
show_summary() {
    print_status "Build Summary:"
    echo "  Build Type: $BUILD_TYPE"
    echo "  Build Directory: $BUILD_DIR"
    echo "  Target: $TARGET"
    echo "  Tests: $BUILD_TESTS"
    echo "  Debug Logging: $ENABLE_DEBUG"
    echo "  Individual Targets: $BUILD_INDIVIDUAL"
    echo "  Vision Support: $BUILD_VISION"
    echo "  Jobs: $JOBS"

    if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
        echo ""
        print_status "Build artifacts:"
        find "$BUILD_DIR" -name "*.a" -o -name "robotics-controller" -o -name "test-*" | head -10
        if [ "$(find "$BUILD_DIR" -name "*.a" -o -name "robotics-controller" -o -name "test-*" | wc -l)" -gt 10 ]; then
            echo "  ... and more"
        fi
    fi
}

# Main execution
main() {
    print_status "Starting Robotics Controller modular build..."
    print_status "Project root: $PROJECT_ROOT"
    print_status "Source directory: $SRC_DIR"

    check_prerequisites
    clean_build
    configure_cmake
    build_project
    show_summary

    print_success "Build process completed!"

    if [ "$TARGET" = "all" ] && [ -f "$BUILD_DIR/robotics-controller" ]; then
        print_status "You can now run: $BUILD_DIR/robotics-controller"
    fi
}

# Run main function
main "$@"
