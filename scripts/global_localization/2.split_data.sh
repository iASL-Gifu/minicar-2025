#!/bin/bash

# ワークスペースのパス（適宜変更）
WS_PATH=/workspaces

# ROS 2 環境を読み込み
source ${WS_PATH}/install/setup.bash

# /workspaces/record/ 配下で日付形式の*.mcapファイルを検索
RECORD_DIR="/workspaces/record"

echo "=========================================="
echo "mcap バッグファイルを検索中..."
echo "=========================================="
echo ""

# camera_pose_*.mcapファイルを含むディレクトリを検索（サブディレクトリも含む）
DIRS=($(find "$RECORD_DIR" -name "camera_pose_*.mcap" -type f 2>/dev/null | xargs -I {} dirname {} | sort -r -u))

# ディレクトリがない場合は終了
if [ ${#DIRS[@]} -eq 0 ]; then
    echo "mcap ファイルが見つかりません"
    exit 1
fi

# メニューを表示
echo "mcap ファイルが存在するディレクトリ:"
echo "=========================================="
for i in "${!DIRS[@]}"; do
    # パスを見やすく表示
    display_path="${DIRS[$i]#$RECORD_DIR/}"
    # ディレクトリ内のmcapファイルを表示
    mcap_file=$(ls "${DIRS[$i]}"/camera_pose_*.mcap 2>/dev/null | head -1)
    if [ -n "$mcap_file" ]; then
        mcap_name=$(basename "$mcap_file")
        echo "$((i + 1)). $display_path / $mcap_name"
    else
        echo "$((i + 1)). $display_path"
    fi
done
echo ""

# ユーザーに選択させる
while true; do
    read -p "分割するディレクトリの番号を入力してください (1-${#DIRS[@]}): " choice
    
    if [[ ! $choice =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt ${#DIRS[@]} ]; then
        echo "無効な入力です。1から${#DIRS[@]}の間で入力してください"
        continue
    fi
    
    break
done

# 選択されたディレクトリに移動
SELECTED_DIR="${DIRS[$((choice - 1))]}"
SELECTED_DIR_PARENT=$(dirname "$SELECTED_DIR")
cd "$SELECTED_DIR_PARENT" || exit 1

echo ""
echo "=========================================="
echo "移動先: $(pwd)"
echo "=========================================="
echo ""

# mcap ファイルを確認（サブディレクトリ内を指定）
MCAP_FILE=$(ls "$SELECTED_DIR"/camera_pose_*.mcap 2>/dev/null | head -1)

if [ -z "$MCAP_FILE" ]; then
    echo "エラー: mcap ファイルが見つかりません"
    exit 1
fi

echo "分割対象: $MCAP_FILE"
echo ""
echo "=========================================="
echo "バッグ分割開始"
echo "=========================================="
echo ""

# 分割1: camera情報（infra1, infra2, camera_info）
echo "分割1: カメラ関連トピックを抽出中..."
ros2 bag filter -o camera_only/ "$SELECTED_DIR" \
  -i "/right/camera_info_rect_fixed" \
     "/left/camera_info_rect" \
     "/infra1/image_rect_raw_mono" \
     "/infra2/image_rect_raw_mono" \
     "/tf_static" \
     "/tf" \
  -s mcap

echo ""

# 分割2: pose情報（tf, tf_static, slam_path）
echo "分割2: ポーズ関連トピックを抽出中..."
ros2 bag filter -o pose_only/ "$SELECTED_DIR" \
  -i "/visual_slam/tracking/slam_path" \
  -s mcap

echo ""
echo "=========================================="
echo "分割完了"
echo "=========================================="
echo "生成されたディレクトリ:"
echo "  - camera_only/  (カメラ関連トピック)"
echo "  - pose_only/    (ポーズ関連トピック)"
echo "=========================================="