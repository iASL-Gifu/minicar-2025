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
echo "  3) Plot and select section points"
echo "  4) Exit"
echo ""
read -p "Enter your choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo -e "${YELLOW}=== Select Map for Odometry Recording ===${NC}"
        echo ""
        
        # マップディレクトリのベースパス
        MAP_BASE_DIR="/workspaces/src/launch/localization_launch/map"
        
        # 利用可能なマップをリスト取得
        if [ -d "$MAP_BASE_DIR" ]; then
            mapfile -t AVAILABLE_MAPS < <(ls -1 "$MAP_BASE_DIR" 2>/dev/null | sort -r)
        fi
        
        # マップ選択
        if [ ${#AVAILABLE_MAPS[@]} -gt 0 ]; then
            echo "Available maps:"
            echo ""
            for i in "${!AVAILABLE_MAPS[@]}"; do
                echo "  $((i+1))) ${AVAILABLE_MAPS[$i]}"
            done
            echo "  0) Enter custom name"
            echo ""
            
            read -p "Select map [0-${#AVAILABLE_MAPS[@]}]: " MAP_CHOICE
            
            if [ "$MAP_CHOICE" = "0" ]; then
                # カスタム名入力
                DEFAULT_MAP_NAME=$(date +%Y%m%d_%H%M%S)
                echo ""
                echo "Default map name: $DEFAULT_MAP_NAME"
                read -p "Enter map name (press Enter to use default): " USER_MAP_NAME
                
                if [ -z "$USER_MAP_NAME" ]; then
                    MAP_NAME="$DEFAULT_MAP_NAME"
                else
                    MAP_NAME="$USER_MAP_NAME"
                fi
            elif [ "$MAP_CHOICE" -ge 1 ] && [ "$MAP_CHOICE" -le ${#AVAILABLE_MAPS[@]} ]; then
                # リストから選択
                MAP_NAME="${AVAILABLE_MAPS[$((MAP_CHOICE-1))]}"
            else
                echo -e "${RED}Invalid selection!${NC}"
                exit 1
            fi
        else
            # マップが見つからない場合
            echo "No existing maps found."
            echo ""
            DEFAULT_MAP_NAME=$(date +%Y%m%d_%H%M%S)
            echo "Default map name: $DEFAULT_MAP_NAME"
            read -p "Enter map name (press Enter to use default): " USER_MAP_NAME
            
            if [ -z "$USER_MAP_NAME" ]; then
                MAP_NAME="$DEFAULT_MAP_NAME"
            else
                MAP_NAME="$USER_MAP_NAME"
            fi
        fi
        
        echo ""
        echo -e "${GREEN}Selected map: ${MAP_NAME}${NC}"
        
        # マップ用のサブディレクトリを作成
        MAP_PATH="$BASE_PATH/$MAP_NAME"
        mkdir -p "$MAP_PATH"
        
        echo ""
        echo -e "${YELLOW}Starting odometry logger...${NC}"
        echo -e "${YELLOW}Data will be saved to: ${MAP_PATH}/${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop and save data${NC}"
        echo ""
        sleep 1
        
        # ロガーを実行（MAP_PATHを環境変数として渡す）
        BASE_PATH="$MAP_PATH" python3 /scripts/global_localization/odom_logger.py
        
        echo ""
        echo -e "${GREEN}Data saved successfully!${NC}"
        ;;
        
    2)
        echo ""
        echo -e "${YELLOW}=== Select Map Directory ===${NC}"
        echo ""
        
        # マップディレクトリをリスト表示
        map_dirs=("$BASE_PATH"/*)
        
        if [ ${#map_dirs[@]} -eq 0 ] || [ ! -d "${map_dirs[0]}" ]; then
            echo -e "${RED}No map directories found in ${BASE_PATH}!${NC}"
            echo "Please run the logger first (option 1)"
            exit 1
        fi
        
        # ディレクトリ一覧を表示
        echo "Available map directories:"
        echo ""
        for i in "${!map_dirs[@]}"; do
            dir="${map_dirs[$i]}"
            if [ -d "$dir" ]; then
                dirname=$(basename "$dir")
                csv_count=$(find "$dir" -maxdepth 1 -name "odometry_path_*.csv" 2>/dev/null | wc -l)
                echo "  $((i+1))) $dirname (${csv_count} CSV files)"
            fi
        done
        
        echo ""
        read -p "Select map directory [1-${#map_dirs[@]}]: " dir_choice
        
        # 入力チェック
        if ! [[ "$dir_choice" =~ ^[0-9]+$ ]] || [ "$dir_choice" -lt 1 ] || [ "$dir_choice" -gt ${#map_dirs[@]} ]; then
            echo -e "${RED}Invalid selection!${NC}"
            exit 1
        fi
        
        # 選択されたディレクトリ
        selected_dir="${map_dirs[$((dir_choice-1))]}"
        
        echo ""
        echo -e "${YELLOW}Available CSV files in $(basename "$selected_dir"):${NC}"
        echo ""
        
        # CSVファイルをリスト表示
        csv_files=("$selected_dir"/odometry_path_*.csv)
        
        if [ ${#csv_files[@]} -eq 0 ] || [ ! -e "${csv_files[0]}" ]; then
            echo -e "${RED}No CSV files found in this directory!${NC}"
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
        echo -e "${YELLOW}=== Select Map Directory ===${NC}"
        echo ""
        
        # マップディレクトリをリスト表示
        map_dirs=("$BASE_PATH"/*)
        
        if [ ${#map_dirs[@]} -eq 0 ] || [ ! -d "${map_dirs[0]}" ]; then
            echo -e "${RED}No map directories found in ${BASE_PATH}!${NC}"
            echo "Please run the logger first (option 1)"
            exit 1
        fi
        
        # ディレクトリ一覧を表示
        echo "Available map directories:"
        echo ""
        for i in "${!map_dirs[@]}"; do
            dir="${map_dirs[$i]}"
            if [ -d "$dir" ]; then
                dirname=$(basename "$dir")
                csv_count=$(find "$dir" -maxdepth 1 -name "odometry_path_*.csv" 2>/dev/null | wc -l)
                echo "  $((i+1))) $dirname (${csv_count} CSV files)"
            fi
        done
        
        echo ""
        read -p "Select map directory [1-${#map_dirs[@]}]: " dir_choice
        
        # 入力チェック
        if ! [[ "$dir_choice" =~ ^[0-9]+$ ]] || [ "$dir_choice" -lt 1 ] || [ "$dir_choice" -gt ${#map_dirs[@]} ]; then
            echo -e "${RED}Invalid selection!${NC}"
            exit 1
        fi
        
        # 選択されたディレクトリ
        selected_dir="${map_dirs[$((dir_choice-1))]}"
        
        echo ""
        echo -e "${YELLOW}Available CSV files in $(basename "$selected_dir"):${NC}"
        echo ""
        
        # CSVファイルをリスト表示
        csv_files=("$selected_dir"/odometry_path_*.csv)
        
        if [ ${#csv_files[@]} -eq 0 ] || [ ! -e "${csv_files[0]}" ]; then
            echo -e "${RED}No CSV files found in this directory!${NC}"
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
        
        # 出力ファイル名を生成（入力ファイル名_section.csv）
        # 例: odometry_path_20250123_120000.csv -> odometry_path_20250123_120000_section.csv
        input_basename=$(basename "$selected_file" .csv)
        output_file="${selected_dir}/${input_basename}_section.csv"
        
        echo ""
        echo -e "${YELLOW}Plotting: $(basename "$selected_file")${NC}"
        echo -e "${YELLOW}Section points will be saved to: $(basename "$output_file")${NC}"
        echo ""
        sleep 1
        
        # プロッターを実行（出力ファイル名を指定）
        python3 /scripts/global_localization/plot_odometry_path.py "$selected_file" 10 "$output_file"
        
        # 保存されたファイルを確認
        if [ -f "$output_file" ]; then
            points=$(wc -l < "$output_file")
            points=$((points - 1))  # ヘッダーを除く
            echo ""
            echo -e "${GREEN}✓ Section points saved successfully!${NC}"
            echo -e "${GREEN}  File: $(basename "$output_file")${NC}"
            echo -e "${GREEN}  Location: $selected_dir${NC}"
            echo -e "${GREEN}  Points: $points${NC}"
        else
            echo ""
            echo -e "${YELLOW}No section points were saved.${NC}"
        fi
        ;;
        
    4)
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