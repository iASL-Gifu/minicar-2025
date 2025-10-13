#!/bin/bash
# Localization起動スクリプト

cd ~/minicar-2025/ros2_ws

echo "========================================="
echo "Localization Launcher"
echo "========================================="

# セットアップスクリプトをソース
echo "Sourcing ROS2 workspace..."
source install/setup.bash

echo ""
echo "Launching Localization..."
echo "========================================="

# Localizationを起動
ros2 launch localization_launch localization.launch.xml

echo ""
echo "========================================="
echo "Localization stopped."
echo "========================================="