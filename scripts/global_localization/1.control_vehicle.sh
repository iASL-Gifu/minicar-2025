#!/bin/bash

# ワークスペースパス（必要に応じて変更）
WS_PATH=/workspaces

# ROS 2 環境設定を読み込み
source ${WS_PATH}/install/setup.bash

ros2 launch system_launch base_system.launch.xml \
  teleop_param:=$(ros2 pkg prefix joy_manager)/share/joy_manager/config/global_localization.yaml