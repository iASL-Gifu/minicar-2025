#!/bin/bash
# ワークスペースのパス
WS_PATH=/workspaces
# ROS 2 環境を読み込み
source ${WS_PATH}/install/setup.bash
# 固定値
POSE_TOPIC_NAME="/visual_slam/tracking/slam_path"
# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# カメラ設定ファイルのデフォルトパス
DEFAULT_CAMERA_CONFIG="$SCRIPT_DIR/camera_config.yaml"
CAMERA_CONFIG_FILE="$DEFAULT_CAMERA_CONFIG"
echo "=========================================="
echo "rosbag フィーチャー抽出ツール"
echo "=========================================="
echo ""
# record ディレクトリを検索
BASE_RECORD_DIR="/workspaces/record"
# 日付形式のディレクトリを検索
DIRS=($(find "$BASE_RECORD_DIR" -maxdepth 1 -type d -name "[0-9]*" | sort -r))
if [ ${#DIRS[@]} -eq 0 ]; then
echo "エラー: record ディレクトリが見つかりません: $BASE_RECORD_DIR"
exit 1
fi
# メニューを表示
echo "利用可能な record ディレクトリ:"
echo "=========================================="
for i in "${!DIRS[@]}"; do
 display_path="${DIRS[$i]#$BASE_RECORD_DIR/}"
echo "$((i + 1)). $display_path"
done
echo ""
# ユーザーに選択させる
while true; do
read -p "使用するディレクトリの番号を入力してください (1-${#DIRS[@]}): " choice
if [[ ! $choice =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt ${#DIRS[@]} ]; then
echo "無効な入力です。1から${#DIRS[@]}の間で入力してください"
continue
fi
break
done
# 選択されたディレクトリ
RECORD_DIR="${DIRS[$((choice - 1))]}"
# バッグディレクトリ名を取得（例: 20251018_113716）
BAG_DIR_NAME=$(basename "$RECORD_DIR")
echo ""
echo "選択されたディレクトリ: $RECORD_DIR"
echo "バッグディレクトリ名: $BAG_DIR_NAME"
echo ""
# camera_only と pose_only ディレクトリを確認
CAMERA_ONLY_DIR="$RECORD_DIR/camera_only"
POSE_ONLY_DIR="$RECORD_DIR/pose_only"
if [ ! -d "$CAMERA_ONLY_DIR" ]; then
echo "エラー: camera_only ディレクトリが見つかりません: $CAMERA_ONLY_DIR"
exit 1
fi
if [ ! -d "$POSE_ONLY_DIR" ]; then
echo "エラー: pose_only ディレクトリが見つかりません: $POSE_ONLY_DIR"
exit 1
fi
# camera_only 内のmcapファイルを検索
CAMERA_BAG=$(find "$CAMERA_ONLY_DIR" -maxdepth 1 -name "*_0.mcap" -type f | head -1)
if [ -z "$CAMERA_BAG" ]; then
echo "エラー: camera_only ディレクトリ内にmcapファイルが見つかりません"
exit 1
fi
# pose_only 内のmcapファイルを検索
POSE_BAG=$(find "$POSE_ONLY_DIR" -maxdepth 1 -name "*_0.mcap" -type f | head -1)
if [ -z "$POSE_BAG" ]; then
echo "エラー: pose_only ディレクトリ内にmcapファイルが見つかりません"
exit 1
fi
# 出力フォルダのベースパス
BASE_OUTPUT_FOLDER="/workspaces/src/launch/localization_launch/keyframes"
# 最終的な出力フォルダパスを構築
MAP_FOLDER="$BASE_OUTPUT_FOLDER/$BAG_DIR_NAME/keyframes"
echo "=========================================="
echo "入力値の確認"
echo "=========================================="
echo "センサーデータ bag: $CAMERA_BAG"
echo "ポーズ bag: $POSE_BAG"
echo "出力フォルダ: $MAP_FOLDER"
echo "ポーズトピック: $POSE_TOPIC_NAME"
if [ -n "$CAMERA_CONFIG_FILE" ]; then
echo "カメラ設定ファイル: $CAMERA_CONFIG_FILE"
fi
echo "=========================================="
echo ""
# ファイルの存在確認
if [ ! -f "$CAMERA_BAG" ]; then
echo "エラー: センサーデータ bag ファイルが見つかりません: $CAMERA_BAG"
exit 1
fi
if [ ! -f "$POSE_BAG" ]; then
echo "エラー: ポーズ bag ファイルが見つかりません: $POSE_BAG"
exit 1
fi
if [ -n "$CAMERA_CONFIG_FILE" ] && [ ! -f "$CAMERA_CONFIG_FILE" ]; then
echo "エラー: カメラ設定ファイルが見つかりません: $CAMERA_CONFIG_FILE"
exit 1
fi
# 出力フォルダを作成
mkdir -p "$MAP_FOLDER"
echo "フィーチャー抽出を開始します..."
echo ""
# コマンドを構築
CMD="ros2 run isaac_mapping_ros rosbag_to_mapping_data \
 --sensor_data_bag_file=$CAMERA_BAG \
 --pose_bag_file=$POSE_BAG \
 --output_folder_path=$MAP_FOLDER \
 --extract_feature --min_inter_frame_rotation_degrees=5 --min_inter_frame_distance=0.2 \
 --pose_topic_name=$POSE_TOPIC_NAME \
 --keypoint_creation_config=\$(ros2 pkg prefix isaac_mapping_ros)/share/isaac_mapping_ros/configs/keypoint_creation_config.pb.txt"
# カメラ設定ファイルがあれば追加
if [ -n "$CAMERA_CONFIG_FILE" ]; then
 CMD="$CMD \
 --camera_topic_config=$CAMERA_CONFIG_FILE"
fi
# コマンド実行
eval "$CMD"
echo ""
echo "=========================================="
echo "フィーチャー抽出完了"
echo "=========================================="
echo "出力フォルダ: $MAP_FOLDER"
echo "=========================================="