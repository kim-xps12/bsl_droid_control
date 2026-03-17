#!/bin/bash
# =============================================================================
# cpp-lint-check.sh - Claude Code PostToolUse hook for C++ code quality
# =============================================================================
# このスクリプトはEdit/Writeツール実行後に自動でclang-formatとcpplintを実行する
#
# 使用方法:
#   stdinからtool_inputのJSONを受け取り、file_pathを抽出して検査を実行
#
# 終了コード:
#   0: 成功（警告があってもフィードバックとして出力）
#   非0: hookの実行自体に失敗
# =============================================================================

set -euo pipefail

PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(pwd)}"
ROS2_WS_DIR="$PROJECT_DIR/ros2_ws"

# stdinからJSONを読み取り
INPUT_JSON=$(cat)

# file_pathを抽出
FILE_PATH=$(echo "$INPUT_JSON" | jq -r '.tool_input.file_path // empty' 2>/dev/null || true)

# ファイルパスが空の場合は終了
if [ -z "$FILE_PATH" ]; then
    exit 0
fi

# C++ファイル以外は対象外
case "$FILE_PATH" in
    *.cpp|*.hpp|*.h) ;;
    *) exit 0 ;;
esac

# robstride_hardware以下のファイルのみ対象
if [[ ! "$FILE_PATH" == *"ros2_ws/src/robstride_hardware/"* ]]; then
    exit 0
fi

# ファイルが存在するか確認
if [ ! -f "$FILE_PATH" ]; then
    exit 0
fi

echo "=== C++ Quality Check: $(basename "$FILE_PATH") ==="

# clang-formatチェック
echo ""
echo "[clang-format] Checking..."
cd "$ROS2_WS_DIR"
FORMAT_OUTPUT=$(pixi run clang-format --dry-run --Werror "$FILE_PATH" 2>&1 || true)
if [ -z "$FORMAT_OUTPUT" ]; then
    echo "[clang-format] No formatting issues."
else
    echo "$FORMAT_OUTPUT"
    echo "[clang-format] Formatting issues detected. Run: cd ros2_ws && pixi run format-cpp"
fi

# cpplintチェック
echo ""
echo "[cpplint] Checking..."
CPPLINT_OUTPUT=$(pixi run ament_cpplint --linelength 100 --filters=-build/header_guard "$FILE_PATH" 2>&1 || true)
if echo "$CPPLINT_OUTPUT" | grep -q "No problems found"; then
    echo "[cpplint] No issues found."
else
    echo "$CPPLINT_OUTPUT" | grep -v "^Done processing" | grep -v "^Using " | grep -v "^$" || true
    if echo "$CPPLINT_OUTPUT" | grep -q "Total errors found: 0"; then
        echo "[cpplint] No issues found."
    fi
fi

echo ""
echo "=== C++ Quality Check Complete ==="
