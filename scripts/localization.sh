#!/bin/bash

# ワークスペースのパス
WS_PATH=/workspaces

# ROS 2 環境を読み込み
source ${WS_PATH}/install/setup.bash

echo "=========================================="
echo "Visual SLAM Launch: Localization Mode"
echo "=========================================="
echo ""

# Base directories
MAP_BASE_DIR="/workspaces/src/launch/localization_launch/map"
KEYFRAMES_BASE_DIR="/workspaces/src/launch/localization_launch/keyframes"

# 利用可能なマップを取得
if [ -d "$MAP_BASE_DIR" ]; then
  mapfile -t AVAILABLE_MAPS < <(ls -1 "$MAP_BASE_DIR" 2>/dev/null | sort -r)
fi

if [ ${#AVAILABLE_MAPS[@]} -eq 0 ]; then
  echo "ERROR: No maps found in $MAP_BASE_DIR"
  exit 1
fi

# ユーザーにマップ選択させる
echo "Available maps:"
for i in "${!AVAILABLE_MAPS[@]}"; do
  echo "  $((i+1))) ${AVAILABLE_MAPS[$i]}"
done
echo ""

read -p "Select map [1-${#AVAILABLE_MAPS[@]}]: " MAP_CHOICE

if [ "$MAP_CHOICE" -ge 1 ] && [ "$MAP_CHOICE" -le ${#AVAILABLE_MAPS[@]} ]; then
  MAP_NAME="${AVAILABLE_MAPS[$((MAP_CHOICE-1))]}"
else
  echo "Invalid selection. Exiting."
  exit 1
fi

# パス設定
LOAD_MAP_PATH="$MAP_BASE_DIR/$MAP_NAME"
VGL_MAP_DIR="$KEYFRAMES_BASE_DIR/$MAP_NAME"

# 存在確認
if [ ! -d "$LOAD_MAP_PATH" ]; then
  echo "ERROR: Map directory does not exist: $LOAD_MAP_PATH"
  exit 1
fi

if [ ! -d "$VGL_MAP_DIR" ]; then
  echo "ERROR: Keyframes directory does not exist: $VGL_MAP_DIR"
  exit 1
fi

# Launch 引数設定
LAUNCH_ARGS="launch_global_localization:=true load_map_path:=$LOAD_MAP_PATH vgl_map_dir:=$VGL_MAP_DIR"

echo ""
echo "=========================================="
echo "Launch Configuration:"
echo "  Mode: Localization"
echo "  Map name: $MAP_NAME"
echo "  Load map from: $LOAD_MAP_PATH"
echo "  Keyframes from: $VGL_MAP_DIR"
echo "=========================================="
echo ""

read -p "Press Enter to launch, or Ctrl+C to cancel..."

# Launch
ros2 launch localization_launch all.launch.xml $LAUNCH_ARGS
