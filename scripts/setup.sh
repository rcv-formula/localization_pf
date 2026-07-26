#!/usr/bin/env bash
#
# Fetch dependencies and build localization_pf.
#
#   - range_libc : git submodule vendored under third_party/ (this repo)
#   - vesc_msgs  : sparse-checkout from rcv-formula/f1_stack_for_damvi
#                  (only vesc/vesc_msgs is pulled, into <ws>/src/vesc_msgs)
#
# Run from anywhere; the script locates the ROS 2 workspace from its own path,
# assuming the standard layout  <ws>/src/localization_pf/scripts/setup.sh
#
# Usage:
#   ./scripts/setup.sh            # fetch deps and build
#   ./scripts/setup.sh --no-build # fetch deps only
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$HERE")"        # <ws>/src/localization_pf
SRC_DIR="$(dirname "$PKG_DIR")"     # <ws>/src
WS_DIR="$(dirname "$SRC_DIR")"      # <ws>

VESC_REPO="https://github.com/rcv-formula/f1_stack_for_damvi.git"
VESC_SUBPATH="vesc/vesc_msgs"

echo "[1/3] range_libc submodule ..."
git -C "$PKG_DIR" submodule update --init --recursive

echo "[2/3] vesc_msgs (from f1_stack_for_damvi) ..."
if [ -d "$SRC_DIR/vesc_msgs" ]; then
  echo "      vesc_msgs already present in $SRC_DIR — leaving it as is."
else
  TMP="$(mktemp -d)"
  git clone --quiet --depth 1 --filter=blob:none --sparse "$VESC_REPO" "$TMP"
  git -C "$TMP" sparse-checkout set "$VESC_SUBPATH"
  cp -r "$TMP/$VESC_SUBPATH" "$SRC_DIR/vesc_msgs"
  rm -rf "$TMP"
  echo "      -> $SRC_DIR/vesc_msgs"
fi

if [ "${1:-}" = "--no-build" ]; then
  echo "Dependencies ready (skipped build)."
  exit 0
fi

echo "[3/3] colcon build ..."
cd "$WS_DIR"
colcon build --packages-select vesc_msgs localization_pf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "Done. Source the overlay before running:"
echo "  source $WS_DIR/install/setup.bash"
echo "  ros2 launch localization_pf localization_pf.launch.py"
