#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

cd "$WORKSPACE_DIR"

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
"$WORKSPACE_DIR/scripts/merge_compile_commands.sh"

echo
echo "VSCode compile database is up to date."
echo "If diagnostics are still stale, run: Ctrl+Shift+P -> clangd: Restart language server"
