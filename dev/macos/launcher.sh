#!/bin/sh

DIR="$(cd "$(dirname "$0")" && pwd)"
USER_DIR="$HOME/Library/Application Support/VoxelCore"

mkdir -p "$USER_DIR"
exec "$DIR/VoxelCore-bin" --res "$DIR/../Resources/res" --dir "$USER_DIR" "$@"
