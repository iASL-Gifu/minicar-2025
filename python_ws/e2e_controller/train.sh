#!/bin/bash

# --- スクリプト設定 ---
TRAIN_SCRIPT_NAME="2_train.py"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
TRAIN_SCRIPT_PATH="${SCRIPT_DIR}/${TRAIN_SCRIPT_NAME}"

# --- 色付け (オプション) ---
CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# --- ヘルプ関数 ---
show_help() {
    echo "Usage: $0 -d <path>"
    echo ""
    echo "Starts the training process using a pre-built dataset."
    echo ""
    echo "Options:"
    echo "  -d, --data_path   Path to the root directory of the dataset (e.g., ./datasets)"
    echo "  -h, --help        Show this help message"
}

# --- 引数解析 ---
DATA_PATH=""

while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -d|--data_path)
        DATA_PATH="$2"
        shift 2
        ;;
        -h|--help)
        show_help
        exit 0
        ;;
        *)
        echo "Unknown option: $1"
        show_help
        exit 1
        ;;
    esac
done

# 必須引数のチェック
if [ -z "$DATA_PATH" ]; then
    echo -e "${RED}ERROR: -d (--data_path) is a required argument.${NC}"
    show_help
    exit 1
fi

# Pythonスクリプトの存在チェック
if [ ! -f "$TRAIN_SCRIPT_PATH" ]; then
    echo -e "${RED}CRITICAL ERROR: Training script not found at: $TRAIN_SCRIPT_PATH${NC}"
    exit 1
fi
if [ ! -d "$DATA_PATH" ]; then
    echo -e "${RED}ERROR: Dataset directory not found at: $DATA_PATH${NC}"
    exit 1
fi

# --- 学習実行 ---
echo -e "\n🚀🚀🚀 Starting training process... 🚀🚀🚀"
echo -e "   Using dataset path: ${CYAN}$DATA_PATH${NC}"

python3 "$TRAIN_SCRIPT_PATH" data_path="$DATA_PATH"

if [ $? -eq 0 ]; then
    echo -e "\n🎉🎉🎉 Training finished successfully! 🎉🎉🎉"
else
    echo -e "\n${RED}❌ ERROR: Training script failed.${NC}"
    exit 1
fi
