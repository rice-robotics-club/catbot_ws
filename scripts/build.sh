#!/bin/bash
set -e
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo "Building all packages..."
colcon build --symlink-install
source "$WORKSPACE_DIR/install/setup.bash"
echo "Build complete. source/install/setup.bash has been sourced."
echo "You can now run the launch and monitor scripts."
