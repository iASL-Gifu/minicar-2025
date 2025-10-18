#!/bin/bash

# ワークスペースパス（必要に応じて変更）
WS_PATH=/workspaces

# ROS 2 環境設定を読み込み
source ${WS_PATH}/install/setup.bash

# Bag Manager ノードを起動（パラメータファイルを上書き指定）
ros2 launch bag_manager_py bag_manager_node.launch.xml \
  bag_manager_param:=$(ros2 pkg prefix bag_manager_py)/share/bag_manager_py/config/global_localization/bag_manager.param.yaml
