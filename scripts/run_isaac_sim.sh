#!/usr/bin/env bash
# scripts/run_isaac_sim.sh
# ─────────────────────────────────────────────────────────────────────────────
# Launch the Isaac Sim simulation with the TurtleBot-like robot.
#
# Supports two Isaac Sim install methods:
#
#   1. Omniverse Launcher / NGC container (python.sh wrapper)
#        Set ISAAC_SIM_ROOT to the install directory, or let the script
#        search the common default paths.
#        e.g.  export ISAAC_SIM_ROOT=~/.local/share/ov/pkg/isaac-sim-5.1.0
#
#   2. NVIDIA PyPI (pip install isaacsim ...)
#        No ISAAC_SIM_ROOT needed — the script detects that `isaacsim` is
#        importable and falls back to plain `python3.11`.
#        e.g.  pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
#
# Usage:
#   bash scripts/run_isaac_sim.sh [--headless]
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# ── 1. Try Omniverse Launcher / NGC install (python.sh) ──────────────────────
PYTHON_CMD=""

if [[ -z "${ISAAC_SIM_ROOT:-}" ]]; then
    # Common default paths (adjust for your OS / install method)
    CANDIDATES=(
        "$HOME/.local/share/ov/pkg/isaac-sim-5.1.0"
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

if [[ -n "${ISAAC_SIM_ROOT:-}" && -f "${ISAAC_SIM_ROOT}/python.sh" ]]; then
    PYTHON_CMD="${ISAAC_SIM_ROOT}/python.sh"
    echo "Install method : Omniverse Launcher / NGC"
    echo "Isaac Sim root : $ISAAC_SIM_ROOT"
fi

# ── 2. Fall back to PyPI install (plain python3.11) ──────────────────────────
if [[ -z "$PYTHON_CMD" ]]; then
    if python3.11 -c "import isaacsim" 2>/dev/null; then
        PYTHON_CMD="python3.11"
        echo "Install method : NVIDIA PyPI (pip)"
    fi
fi

# ── Give up if neither install is found ──────────────────────────────────────
if [[ -z "$PYTHON_CMD" ]]; then
    echo "ERROR: Isaac Sim not found via either install method."
    echo ""
    echo "Option A — Omniverse Launcher / NGC:"
    echo "  Set ISAAC_SIM_ROOT to your Isaac Sim install directory, e.g.:"
    echo "    export ISAAC_SIM_ROOT=\$HOME/.local/share/ov/pkg/isaac-sim-5.1.0"
    echo ""
    echo "Option B — NVIDIA PyPI:"
    echo "  pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com"
    exit 1
fi

# ── Set ROS_DOMAIN_ID to match the Docker container ──────────────────────────
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
echo "ROS_DOMAIN_ID  : $ROS_DOMAIN_ID"

# ── Run the simulation ────────────────────────────────────────────────────────
"$PYTHON_CMD" "$REPO_ROOT/isaac_sim/setup_scene.py" "$@"
