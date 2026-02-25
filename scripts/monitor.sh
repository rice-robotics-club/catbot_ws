#!/bin/bash
# Usage: ./monitor.sh [topic]
# If no topic is given, lists available topics and prompts for selection.
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

source "$WORKSPACE_DIR/install/setup.bash"

TOPIC="$1"

if [ -z "$TOPIC" ]; then
    echo "Available topics:"
    ros2 topic list
    echo ""
    read -p "Enter topic to monitor: " TOPIC
fi

echo "Monitoring $TOPIC..."

# ── Joint-angle visual TUI for /actions ──────────────────────────────────────
# Strip optional leading slash for comparison
TOPIC_BARE="${TOPIC#/}"
if [[ "$TOPIC_BARE" == "actions" ]]; then
    exec python3 "$(dirname "$0")/../utils/joint_monitor.py"
fi

# ── rosshow supports select sensor/nav types but not std_msgs or custom msgs.
# Check the actual type before launching to avoid the "Unsupported message type" dead end.
MSG_TYPE=$(ros2 topic type "$TOPIC" 2>/dev/null)
ROSSHOW_SUPPORTED="sensor_msgs/ nav_msgs/ geometry_msgs/Twist geometry_msgs/Pose"

USE_ROSSHOW=false
for prefix in $ROSSHOW_SUPPORTED; do
    if [[ "$MSG_TYPE" == ${prefix}* ]]; then
        USE_ROSSHOW=true
        break
    fi
done

if $USE_ROSSHOW; then
    exec ros2 run rosshow rosshow "$TOPIC"
else
    echo "  type: $MSG_TYPE (rosshow unsupported — using ros2 topic echo)"
    exec ros2 topic echo "$TOPIC"
fi
