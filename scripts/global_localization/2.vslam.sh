#!/bin/bash

# ワークスペースのパス（適宜変更）
WS_PATH=/workspaces

# ROS 2 環境を読み込み
source ${WS_PATH}/install/setup.bash

# RealSense設定ファイルのパスを指定してlaunch起動
ros2 launch localization_launch vslam.launch.xml