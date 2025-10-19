#!/bin/bash

# recordディレクトリに移動
cd /workspaces/record || exit 1

# 日付でディレクトリ名を生成（例：20251018_104530）
DIR_NAME=$(date +%Y%m%d_%H%M%S)

# ディレクトリを作成
mkdir -p "$DIR_NAME"

# 作成したディレクトリに移動
cd "$DIR_NAME" || exit 1

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