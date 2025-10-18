#!/bin/bash

# recordディレクトリに移動
cd record || exit 1

# rosbagファイルを検索（サブディレクトリも含む）
BAGS=($(find . -type f \( -name "*.mcap" -o -name "*.bag" \) | sort))

# ファイルがない場合は終了
if [ ${#BAGS[@]} -eq 0 ]; then
    echo "rosbagファイルが見つかりません"
    exit 1
fi

# メニューを表示
echo "=========================================="
echo "rosbagファイル一覧"
echo "=========================================="
for i in "${!BAGS[@]}"; do
    # パス表示を見やすくする（./を削除）
    display_path="${BAGS[$i]#./}"
    echo "$((i + 1)). $display_path"
done
echo ""

# ユーザーに選択させる
while true; do
    read -p "再生するrosbagファイルの番号を入力してください (1-${#BAGS[@]}): " choice
    
    # 入力値の検証
    if [[ ! $choice =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt ${#BAGS[@]} ]; then
        echo "無効な入力です。1から${#BAGS[@]}の間で入力してください"
        continue
    fi
    
    break
done

# 選択されたファイル
SELECTED_BAG="${BAGS[$((choice - 1))]}"

echo ""
echo "=========================================="
echo "再生ファイル: ${SELECTED_BAG#./}"
echo "=========================================="
echo ""

# rosbagを再生
ros2 bag play "$SELECTED_BAG"