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

# Genesis: biped_genesis_sim パッケージ（Genesis物理エンジンROS 2ブリッジ）に必要。
# 以前はここで pip による自動インストールを行っていたが、
# activation 時の暗黙インストールは再現性・オフライン環境・権限等の観点から問題があるため、
# 未インストールであることのみ検出し、明示的なセットアップタスクの実行を案内する。
if [ -n "$CONDA_PREFIX" ]; then
    if ! python -c "import genesis" 2>/dev/null; then
        echo "[activate.sh] WARNING: Python モジュール 'genesis' が見つかりません。" >&2
        echo "[activate.sh] biped_genesis_sim を利用する前に、別途セットアップを実行してください。" >&2
        echo "[activate.sh] 例: pixi run setup-genesis" >&2
    fi
fi
