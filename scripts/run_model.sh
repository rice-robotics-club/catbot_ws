#!/bin/bash
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

export CATBOT_WS="$WORKSPACE_DIR"
export CATBOT_CONFIG="$WORKSPACE_DIR/config/robot.yaml"

source "$WORKSPACE_DIR/install/setup.bash"

echo "Starting RL model node..."
exec ros2 run rsl_runner model
