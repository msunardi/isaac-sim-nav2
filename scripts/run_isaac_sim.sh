#!/usr/bin/env bash
# scripts/run_isaac_sim.sh
# ─────────────────────────────────────────────────────────────────────────────
# Launch the Isaac Sim simulation with the TurtleBot-like robot.
#
# Prerequisites:
#   - NVIDIA Isaac Sim 4.x installed (via Omniverse Launcher or .run installer)
#   - ISAAC_SIM_ROOT env var set, OR Isaac Sim installed at the default path
#
# Usage:
#   bash scripts/run_isaac_sim.sh [--headless]
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# ── Locate Isaac Sim root ─────────────────────────────────────────────────────
if [[ -z "${ISAAC_SIM_ROOT:-}" ]]; then
    # Common default paths (adjust for your OS / install method)
    CANDIDATES=(
        "$HOME/.local/share/ov/pkg/isaac-sim-4.5.0"
        "$HOME/.local/share/ov/pkg/isaac-sim-4.2.0"
        "$HOME/.local/share/ov/pkg/isaac-sim-4.1.0"
        "/isaac-sim"
        "/opt/nvidia/isaac-sim"
    )
    for candidate in "${CANDIDATES[@]}"; do
        if [[ -f "$candidate/python.sh" ]]; then
            ISAAC_SIM_ROOT="$candidate"
            break
        fi
    done
fi

if [[ -z "${ISAAC_SIM_ROOT:-}" || ! -f "${ISAAC_SIM_ROOT}/python.sh" ]]; then
    echo "ERROR: Isaac Sim not found."
    echo "  Set ISAAC_SIM_ROOT to your Isaac Sim install directory, e.g.:"
    echo "    export ISAAC_SIM_ROOT=\$HOME/.local/share/ov/pkg/isaac-sim-4.5.0"
    exit 1
fi

echo "Using Isaac Sim at: $ISAAC_SIM_ROOT"

# ── Set ROS_DOMAIN_ID to match the Docker container ──────────────────────────
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# ── Run the simulation ────────────────────────────────────────────────────────
"$ISAAC_SIM_ROOT/python.sh" \
    "$REPO_ROOT/isaac_sim/setup_scene.py" \
    "$@"
