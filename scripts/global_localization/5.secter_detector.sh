#!/bin/bash

# ワークスペースのパス
WS_PATH=/workspaces

# ROS 2 環境を読み込み
source /opt/ros/humble/setup.bash
source ${WS_PATH}/install/setup.bash

# CSVファイルの保存・読み込みディレクトリ
BASE_PATH="/workspaces/src/launch/localization_launch/path"

# ディレクトリが存在しない場合は作成
mkdir -p "$BASE_PATH"

# カラー定義
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# タイトル表示
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  Odometry Tool Manager${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# メニュー表示
echo -e "${GREEN}Select mode:${NC}"
echo "  1) Record odometry data"
echo "  2) Plot odometry data"
echo "  3) Exit"
echo ""
read -p "Enter your choice [1-3]: " choice

case $choice in
    1)
        echo ""
        echo -e "${YELLOW}Starting odometry logger...${NC}"
        echo -e "${YELLOW}Data will be saved to: ${BASE_PATH}/${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop and save data${NC}"
        echo ""
        sleep 1
        
        # ロガーを実行（BASE_PATHを環境変数として渡す）
        BASE_PATH="$BASE_PATH" python3 /scripts/global_localization/odom_logger.py
        
        echo ""
        echo -e "${GREEN}Data saved successfully!${NC}"
        ;;
        
    2)
        echo ""
        echo -e "${YELLOW}Available CSV files in ${BASE_PATH}:${NC}"
        echo ""
        
        # CSVファイルをリスト表示
        csv_files=("$BASE_PATH"/odometry_path_*.csv)
        
        if [ ${#csv_files[@]} -eq 0 ] || [ ! -e "${csv_files[0]}" ]; then
            echo -e "${RED}No CSV files found in ${BASE_PATH}!${NC}"
            echo "Please run the logger first (option 1)"
            exit 1
        fi
        
        # ファイル一覧を表示
        for i in "${!csv_files[@]}"; do
            file="${csv_files[$i]}"
            basename=$(basename "$file")
            size=$(du -h "$file" | cut -f1)
            lines=$(wc -l < "$file")
            points=$((lines - 1))  # ヘッダーを除く
            echo "  $((i+1))) $basename (${size}, ${points} points)"
        done
        
        echo ""
        read -p "Select file number [1-${#csv_files[@]}]: " file_choice
        
        # 入力チェック
        if ! [[ "$file_choice" =~ ^[0-9]+$ ]] || [ "$file_choice" -lt 1 ] || [ "$file_choice" -gt ${#csv_files[@]} ]; then
            echo -e "${RED}Invalid selection!${NC}"
            exit 1
        fi
        
        # 選択されたファイル
        selected_file="${csv_files[$((file_choice-1))]}"
        
        echo ""
        echo -e "${YELLOW}Plotting: $(basename "$selected_file")${NC}"
        echo ""
        sleep 1
        
        # プロッターを実行
        python3 /scripts/global_localization/plot_odometry_path.py "$selected_file"
        
        echo ""
        echo -e "${GREEN}Plot closed.${NC}"
        ;;
        
    3)
        echo ""
        echo -e "${GREEN}Goodbye!${NC}"
        exit 0
        ;;
        
    *)
        echo ""
        echo -e "${RED}Invalid choice!${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Done!${NC}"
echo -e "${BLUE}================================${NC}"