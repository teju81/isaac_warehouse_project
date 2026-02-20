#!/bin/bash
# =============================================================================
# Nav2 Sandbox — Tmux Launcher
# =============================================================================
# Launches all ROS2 navigation components in a single tmux window with 4 panes.
# Run this inside the Docker container (Isaac Sim should already be running on host).
#
# Robot selection is driven by project_config.yaml — change 'robot: <name>'
# to switch between carter, g1, anymal, etc.
#
# Layout:
#   +-------------------------------+-------------------------------+
#   |  1. Sensor Bridge             |  2. Nav2 Stack                |
#   |  localization + scan source   |  planner, controller, etc.    |
#   +-------------------------------+-------------------------------+
#   |  3. RViz                      |  4. Interactive Shell         |
#   |  visualization                |  teleop, goals, monitoring    |
#   +-------------------------------+-------------------------------+
#
# Usage:
#   ./launch_tmux.sh                                    # uses map from project_config.yaml
#   ./launch_tmux.sh --map /path/to/map_auto.yaml       # override map path
#
# Tmux Controls:
#   Ctrl+b, arrow keys  — Navigate between panes
#   Ctrl+b, z           — Zoom current pane full-screen (toggle)
#   Ctrl+b, d           — Detach from session (keeps running)
#   Ctrl+b, x           — Kill current pane
#   tmux attach -t nav2  — Reattach to session
# =============================================================================

set -e

SESSION_NAME="nav2"
DOMAIN_ID=47

# Clean ALL stale FastDDS shared memory from previous runs.
# fastrtps_* = SHM transport segments
# fastdds_*  = Data sharing (zero-copy) pools
# Stale files cause CDR deserialization errors ("sequence size exceeds
# remaining buffer") even when SHM/data-sharing is disabled in the XML profile.
rm -rf /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true

# Parse arguments (override map path from project_config.yaml)
MAP_ARG=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --map)
            MAP_ARG="map:=$2"
            shift 2
            ;;
        --map=*)
            MAP_ARG="map:=${1#*=}"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

# Detect workspace root (script is in scripts/ or lib/nav2_sandbox/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
    WORKSPACE="$SCRIPT_DIR/../.."
elif [ -f "$SCRIPT_DIR/../../../install/setup.bash" ]; then
    WORKSPACE="$SCRIPT_DIR/../../.."
else
    # Fallback: assume ros2_ws is the workspace
    WORKSPACE="/root/mycode/isaac_warehouse_project/ros2_ws"
fi
WORKSPACE="$(cd "$WORKSPACE" && pwd)"

# Shell setup command (sourced in every pane)
# FASTRTPS_DEFAULT_PROFILES_FILE loads an XML that forces UDPv4-only transport,
# disabling SHM which causes CDR errors across Docker container boundaries.
# NOTE: FASTDDS_BUILTIN_TRANSPORTS env var does NOT work on FastDDS 2.6 (Humble).
FASTDDS_XML="$WORKSPACE/install/nav2_sandbox/share/nav2_sandbox/config/fastdds_profile.xml"
SETUP="source /opt/ros/humble/setup.bash 2>/dev/null; source $WORKSPACE/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=$DOMAIN_ID; export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML"

# Kill existing tmux server to clear stale DDS state (environment, library caches).
# kill-session is not enough — tmux server caches the environment from first session.
tmux kill-server 2>/dev/null || true
sleep 1

# Create session with first pane (top-left)
tmux new-session -d -s "$SESSION_NAME" -x 200 -y 50

# Create 2x2 grid
tmux split-window -h -t "$SESSION_NAME"     # top-right
tmux split-window -v -t "$SESSION_NAME:0.0"  # bottom-left
tmux split-window -v -t "$SESSION_NAME:0.1"  # bottom-right

# ---- Pane 0 (top-left): Sensor Bridge ----
tmux send-keys -t "$SESSION_NAME:0.0" "$SETUP && echo '=== Sensor Bridge ===' && ros2 launch nav2_sandbox sensor_bridge.launch.py" Enter

# ---- Pane 1 (top-right): Nav2 Stack (delayed 5s) ----
tmux send-keys -t "$SESSION_NAME:0.1" "$SETUP && echo '=== Nav2 Stack (starting in 5s) ===' && sleep 5 && ros2 launch nav2_sandbox navigation.launch.py $MAP_ARG" Enter

# ---- Pane 2 (bottom-left): RViz (delayed 8s) ----
tmux send-keys -t "$SESSION_NAME:0.2" "$SETUP && echo '=== RViz (starting in 8s) ===' && sleep 8 && rviz2 -d $WORKSPACE/install/nav2_sandbox/share/nav2_sandbox/config/rviz_nav.rviz --ros-args -p use_sim_time:=true" Enter

# ---- Pane 3 (bottom-right): Interactive shell ----
tmux send-keys -t "$SESSION_NAME:0.3" "$SETUP" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "clear" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo ''" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo '=== Interactive Shell ==='" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo ''" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo 'Available commands:'" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo '  ros2 run nav2_sandbox joy_teleop.py --topic /<ns>/cmd_vel'" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo '  ros2 run nav2_sandbox send_goal.py --x 2.0 --y 1.0'" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo '  ros2 run nav2_sandbox waypoint_follower.py --loop'" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo '  ros2 topic list'" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo ''" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo 'Robot config: see project_config.yaml (robot: <name>)'" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo 'Tmux: Ctrl+b,arrows=navigate | Ctrl+b,z=zoom | Ctrl+b,d=detach'" Enter
tmux send-keys -t "$SESSION_NAME:0.3" "echo ''" Enter

# Select the interactive shell pane
tmux select-pane -t "$SESSION_NAME:0.3"

# Attach to session
tmux attach -t "$SESSION_NAME"
