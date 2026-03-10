#!/bin/sh
# Wrapper script for pixi activation.
# Sources colcon's install/setup.sh if it exists (i.e., after the first build).
# On a fresh clone before the first build, this is a no-op.

# Use PIXI_PROJECT_ROOT (set by pixi) for reliable path resolution.
# $0-based SCRIPT_DIR fails during pixi activation because $0 is the shell binary.
_ACTIVATE_DIR="${PIXI_PROJECT_ROOT:-.}"

if [ -f "$_ACTIVATE_DIR/install/setup.sh" ]; then
    . "$_ACTIVATE_DIR/install/setup.sh"
fi

# Gazebo Harmonic: gz_ros2_control plugin path
if [ -n "$CONDA_PREFIX" ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="${CONDA_PREFIX}/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
fi

# Genesis: pixi pypi-dependenciesではpillow版競合により追加不可のため、
# pip install --no-deps で個別インストールする。
# biped_genesis_sim パッケージ（Genesis物理エンジンROS 2ブリッジ）に必要。
if [ -n "$CONDA_PREFIX" ]; then
    if ! python -c "import genesis" 2>/dev/null; then
        echo "[activate.sh] Installing genesis-world into pixi environment..."
        python -m ensurepip --default-pip -q 2>/dev/null
        python -m pip install -q --no-deps genesis-world quadrants 2>/dev/null
    fi
fi
