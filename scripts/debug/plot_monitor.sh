#!/bin/bash

# 使用方法: /scripts/debug/plot_monitor.sh [csv_directory]
# 例: /scripts/debug/plot_monitor.sh
# 例: /scripts/debug/plot_monitor.sh /path/to/csv/files

# 色付き出力用
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# スクリプトのディレクトリを取得（/scripts/debug/）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Pythonスクリプトのパス（このスクリプトと同じ /scripts/debug/ にある）
PYTHON_SCRIPT="${SCRIPT_DIR}/plot_monitor.py"

# CSVディレクトリを指定（デフォルトは /debug）
CSV_DIR="${1:-/debug}"

# ディレクトリが存在するか確認
if [ ! -d "${CSV_DIR}" ]; then
    echo -e "${RED}Error: Directory not found - ${CSV_DIR}${NC}"
    exit 1
fi

# Pythonスクリプトが存在するか確認
if [ ! -f "${PYTHON_SCRIPT}" ]; then
    echo -e "${RED}Error: ${PYTHON_SCRIPT} not found${NC}"
    echo -e "${YELLOW}Expected location: ${PYTHON_SCRIPT}${NC}"
    echo -e "${YELLOW}Please ensure plot_monitor.py is in ${SCRIPT_DIR}${NC}"
    exit 1
fi

# Python3が利用可能か確認
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Error: python3 is not installed${NC}"
    exit 1
fi

echo -e "${GREEN}=== Jetson Monitor グラフ生成ツール ===${NC}"
echo -e "${GREEN}Script Directory: ${YELLOW}${SCRIPT_DIR}${NC}"
echo -e "${GREEN}CSV Directory: ${YELLOW}${CSV_DIR}${NC}\n"

# CSVファイルを検索
csv_files=(${CSV_DIR}/*.csv)

# CSVファイルが見つからない場合
if [ ! -e "${csv_files[0]}" ]; then
    echo -e "${RED}Error: No CSV files found in ${CSV_DIR}${NC}"
    exit 1
fi

# CSVファイルを日付順にソート（新しい順）
mapfile -t csv_files < <(ls -1tr ${CSV_DIR}/*.csv 2>/dev/null | tac)

# CSVファイルを表示
echo -e "${BLUE}Available CSV files:${NC}"
for i in "${!csv_files[@]}"; do
    # ファイルサイズを取得
    file_size=$(du -h "${csv_files[$i]}" | cut -f1)
    # ファイルの更新日時を取得
    file_date=$(stat -c %y "${csv_files[$i]}" 2>/dev/null | cut -d' ' -f1,2 || stat -f "%Sm" -t "%Y-%m-%d %H:%M:%S" "${csv_files[$i]}")
    
    printf "%3d) %-40s [%s] (%s)\n" $((i + 1)) "${csv_files[$i]}" "${file_size}" "${file_date}"
done

# ユーザーに選択を促す
echo ""
while true; do
    read -p "Select CSV file number (1-${#csv_files[@]}): " selection
    
    # 入力が整数か確認
    if ! [[ "$selection" =~ ^[0-9]+$ ]]; then
        echo -e "${RED}Error: Please enter a valid number${NC}"
        continue
    fi
    
    # 範囲チェック
    if [ "$selection" -lt 1 ] || [ "$selection" -gt ${#csv_files[@]} ]; then
        echo -e "${RED}Error: Number out of range. Please select 1-${#csv_files[@]}${NC}"
        continue
    fi
    
    break
done

# 選択したCSVファイルを取得
CSV_FILE="${csv_files[$((selection - 1))]}"

echo -e "\n${GREEN}Selected File: ${YELLOW}${CSV_FILE}${NC}"
echo -e "${YELLOW}Processing...${NC}\n"

# Pythonスクリプトを実行
python3 "${PYTHON_SCRIPT}" "${CSV_FILE}"

if [ $? -eq 0 ]; then
    echo -e "\n${GREEN}✓ Graph generation completed successfully${NC}"
else
    echo -e "\n${RED}✗ Error occurred during graph generation${NC}"
    exit 1
fi