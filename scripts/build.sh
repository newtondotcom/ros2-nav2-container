#!/bin/bash
# Ensure script is run with bash (not sh)
if [ -z "$BASH_VERSION" ]; then
    echo "Error: This script must be run with bash, not sh" >&2
    echo "Please run: bash $0" >&2
    exit 1
fi

# Source logging functions
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source "$SCRIPT_DIR/logging.sh"

# Change to workspace root (parent of scripts directory)
WORKSPACE_ROOT=$(dirname "$SCRIPT_DIR")
cd "$WORKSPACE_ROOT" || fatal "Failed to change to workspace root: $WORKSPACE_ROOT"

# Ensure log directory exists and is writable
if [ ! -w "log" ] 2>/dev/null; then
    if [ -d "log" ]; then
        info "Warning: log directory exists but is not writable. You may need to fix permissions."
        info "Try: sudo chown -R \$USER:\$USER log"
    else
        mkdir -p log 2>/dev/null || info "Warning: Could not create log directory"
    fi
fi

info "Building the project..."
# Build all packages, explicitly including nav2_bringup which is not a dependency of other packages
colcon --log-level info build \
    --symlink-install \
    --mixin debug ccache compile-commands gold \
    --event-handlers "console_direct+" || fatal "Failed to build the project"
    #--cmake-args -DMY_CPP_NODE_FLAG=ON \

info "Build completed successfully"
