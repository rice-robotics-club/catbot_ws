#!/bin/bash
set -e
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

export CATBOT_WS="$WORKSPACE_DIR"
export CATBOT_CONFIG="$WORKSPACE_DIR/config/robot.yaml"

source "$WORKSPACE_DIR/install/setup.bash"

cleanup() {
    echo ""
    echo "Shutting down visualization..."
    kill $(jobs -p) 2>/dev/null
    wait
}
trap cleanup EXIT INT TERM

echo "Starting 3D visualization (requires display)..."
echo "Robot config: $CATBOT_CONFIG"
ros2 launch rsl_runner visualize.launch.py &

echo "Press Ctrl+C to stop."
wait
