#!/bin/bash

# ===== 設定 =====
# ワークスペースのパス（適宜変更）
WS_PATH=/workspaces

# ===== ROS 2 環境を読み込み =====
source ${WS_PATH}/install/setup.bash

# ===== rviz設定ファイルのパスを取得 =====
RVIZ_PATH="$(ros2 pkg prefix localization_launch)/share/localization_launch/rviz/rviz.rviz"

# ===== ファイル存在チェック =====
if [ -f "$RVIZ_PATH" ]; then
  echo "RViz設定ファイルが見つかりました: $RVIZ_PATH"
  echo "RViz2を起動します..."
  rviz2 -d "$RVIZ_PATH"
else
  echo "RViz設定ファイルが見つかりません: $RVIZ_PATH"
  exit 1
fi
