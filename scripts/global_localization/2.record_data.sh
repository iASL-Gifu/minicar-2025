#!/bin/bash

# recordディレクトリに移動
cd /workspaces/record/map_source || exit 1

# デフォルトのディレクトリ名を生成（例：20251018_104530）
DEFAULT_DIR_NAME=$(date +%Y%m%d_%H%M%S)

echo "=========================================="
echo "ROS2 Bag Recording Setup"
echo "=========================================="
echo ""
echo "Default directory name: $DEFAULT_DIR_NAME"
echo ""
read -p "Enter directory name (press Enter to use default): " USER_INPUT

# ユーザー入力があればそれを使用、なければデフォルトを使用
if [ -z "$USER_INPUT" ]; then
  DIR_NAME="$DEFAULT_DIR_NAME"
  echo "Using default: $DIR_NAME"
else
  DIR_NAME="$USER_INPUT"
  echo "Using: $DIR_NAME"
fi

# ディレクトリを作成
mkdir -p "$DIR_NAME"

# 作成したディレクトリに移動
cd "$DIR_NAME" || exit 1

echo ""
echo "=========================================="
echo "Recording directory: $(pwd)"
echo "=========================================="
echo ""

# ros2 bag recordでデータ記録
ros2 bag record -s mcap -o camera_pose \
  /right/camera_info_rect_fixed \
  /left/camera_info_rect \
  /infra1/image_rect_raw_mono \
  /infra2/image_rect_raw_mono \
  /tf_static \
  /tf \
  /visual_slam/tracking/odometry