#!/bin/bash
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

export CATBOT_WS="$WORKSPACE_DIR"
export CATBOT_CONFIG="$WORKSPACE_DIR/config/robot.yaml"

source "$WORKSPACE_DIR/install/setup.bash"

echo "Launching sensor nodes..."
exec ros2 launch catbot_perception sensors.launch.py
