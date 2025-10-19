#!/bin/bash

# ワークスペースのパス（適宜変更）
WS_PATH=/workspaces

# ROS 2 環境を読み込み
source ${WS_PATH}/install/setup.bash

# RealSense設定ファイルのパスを指定してlaunch起動
ros2 launch localization_launch realsense.launch.xml \
  realsense_config_file:=$(ros2 pkg prefix localization_launch)/share/localization_launch/config/realsense.yaml
