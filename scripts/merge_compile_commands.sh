#!/usr/bin/env bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUTPUT_FILE="$WORKSPACE_DIR/compile_commands.json"

echo "[" > "$OUTPUT_FILE"

first=1
find "$WORKSPACE_DIR/build" -name compile_commands.json | while read -r file; do
  if [ ! -f "$file" ]; then
    continue
  fi

  content=$(cat "$file")
  content="${content#[}"
  content="${content%]}"

  if [ -n "$content" ]; then
    if [ $first -eq 0 ]; then
      echo "," >> "$OUTPUT_FILE"
    fi
    echo "$content" >> "$OUTPUT_FILE"
    first=0
  fi
done

echo "]" >> "$OUTPUT_FILE"
echo "Merged compile_commands.json generated at: $OUTPUT_FILE"
