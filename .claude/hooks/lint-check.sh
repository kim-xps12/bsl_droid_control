#!/bin/bash
# =============================================================================
# lint-check.sh - Claude Code PostToolUse hook for Python code quality
# =============================================================================
# このスクリプトはEdit/Writeツール実行後に自動でruffとmypyを実行する
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
RL_WS_DIR="$PROJECT_DIR/rl_ws"

# stdinからJSONを読み取り
INPUT_JSON=$(cat)

# file_pathを抽出
FILE_PATH=$(echo "$INPUT_JSON" | jq -r '.tool_input.file_path // empty' 2>/dev/null || true)

# ファイルパスが空の場合は終了
if [ -z "$FILE_PATH" ]; then
    exit 0
fi

# Pythonファイル以外は対象外
if [[ ! "$FILE_PATH" == *.py ]]; then
    exit 0
fi

# rl_ws以下のファイルのみ対象（ros2_wsやsubmoduleは除外）
if [[ ! "$FILE_PATH" == *"/rl_ws/"* ]] && [[ ! "$FILE_PATH" == "$RL_WS_DIR/"* ]]; then
    exit 0
fi

# genesis_official, mujoco_menagerie, .venvは除外
if [[ "$FILE_PATH" == *"genesis_official"* ]] || \
   [[ "$FILE_PATH" == *"mujoco_menagerie"* ]] || \
   [[ "$FILE_PATH" == *".venv"* ]]; then
    exit 0
fi

# ファイルが存在するか確認
if [ ! -f "$FILE_PATH" ]; then
    exit 0
fi

echo "=== Code Quality Check: $(basename "$FILE_PATH") ==="

# ruffによるlintチェック
echo ""
echo "[Ruff] Checking..."
cd "$RL_WS_DIR"
RUFF_OUTPUT=$(uv run ruff check "$FILE_PATH" 2>&1 | grep -v "^warning:" | grep -v "^Uninstalled" | grep -v "^Installed" | grep -v "^$" || true)
if echo "$RUFF_OUTPUT" | grep -qE "^\s*[A-Z][0-9]+"; then
    echo "$RUFF_OUTPUT"
    echo ""
    echo "[Ruff] Auto-fixing..."
    uv run ruff check --fix "$FILE_PATH" 2>&1 | grep -v "^warning:" | grep -v "^Uninstalled" | grep -v "^Installed" || true

    # 修正後に再度チェック
    RUFF_OUTPUT_AFTER=$(uv run ruff check "$FILE_PATH" 2>&1 | grep -v "^warning:" | grep -v "^Uninstalled" | grep -v "^Installed" | grep -v "^$" || true)
    if echo "$RUFF_OUTPUT_AFTER" | grep -qE "^\s*[A-Z][0-9]+"; then
        echo "[Ruff] Remaining issues (manual fix required):"
        echo "$RUFF_OUTPUT_AFTER"
    else
        echo "[Ruff] All issues auto-fixed."
    fi
else
    echo "[Ruff] No issues found."
fi

# mypyによる型チェック
echo ""
echo "[Mypy] Type checking..."
MYPY_OUTPUT=$(uv run mypy "$FILE_PATH" 2>&1 | grep -v "^warning:" | grep -v "^Uninstalled" | grep -v "^Installed" || true)
if echo "$MYPY_OUTPUT" | grep -q "error:"; then
    echo "$MYPY_OUTPUT"
else
    echo "[Mypy] No type errors found."
fi

echo ""
echo "=== Code Quality Check Complete ==="
