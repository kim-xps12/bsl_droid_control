#!/bin/sh

set -eu

if [ -z "${CONDA_PREFIX:-}" ]; then
    echo "CONDA_PREFIX is not set" >&2
    exit 1
fi

tmp_log="${TMPDIR:-/tmp}/ros2_colcon_build.$$"

cleanup() {
    rm -f "$tmp_log"
}

trap cleanup EXIT HUP INT TERM

status=0
if ! colcon build --symlink-install "$@" --cmake-args "-DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3" >"$tmp_log" 2>&1; then
    status=$?
fi

# Suppress known macOS/conda-forge install_name_tool noise. All other warnings remain visible.
awk '
    /^--- stderr: biped_msgs$/ { suppress_stderr_block = 1; next }
    suppress_stderr_block && /^---$/ { suppress_stderr_block = 0; next }
    suppress_stderr_block { next }
    /install_name_tool: warning: changes being made to the file will invalidate the code signature/ { next }
    /^\[cctools-port\]: generating fake signature/ { next }
    /^  1 package had stderr output: biped_msgs$/ { next }
    { print }
' "$tmp_log"

exit "$status"
