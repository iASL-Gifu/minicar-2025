#!/bin/bash

# --- 設定 ---
REMOTE_USER="tamiya"
REMOTE_TARGET_DIR="/home/tamiya/workspace/minicar-2025/python_ws/ckpts/pilotnet"

# スクリプトが検索するローカルのチェックポイント・ベースディレクトリ
LOCAL_CKPT_BASE_DIR="./ckpts/train"

# --- 色付け (オプション) ---
CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# --- IP選択メニュー ---
ip_select_menu() {
    echo -e "${CYAN}Select Remote IP:${NC}"
    echo "1) 10.42.0.1 (USB Wi-Fi / Jetson)"
    echo "2) 192.168.11.30 (House Wi-Fi / Jetson)"
    echo "3) 手動入力"
    read -p "Enter choice [1-3]: " choice

    case $choice in
        1) REMOTE_IP="10.42.0.1" ;;
        2) REMOTE_IP="192.168.11.30" ;;
        3) read -p "Enter IP manually: " REMOTE_IP ;;
        *) echo "Invalid choice"; exit 1 ;;
    esac

    echo -e "Using remote IP: ${GREEN}${REMOTE_IP}${NC}"
}

ip_select_menu

# --- 最新ディレクトリを検索 ---
echo -e "\n🔄 Searching for the latest checkpoint DIR in: ${CYAN}$LOCAL_CKPT_BASE_DIR${NC}"

LATEST_DIR=$(find "$LOCAL_CKPT_BASE_DIR" -mindepth 1 -maxdepth 3 -type d -print0 | xargs -0 ls -dt | head -n 1)

if [ -z "$LATEST_DIR" ]; then
    echo -e "${RED}ERROR: No checkpoint directory found in $LOCAL_CKPT_BASE_DIR${NC}"
    exit 1
fi

echo -e "📁 Found latest dir: ${GREEN}$(basename "$LATEST_DIR")${NC}"
echo -e "Full path: $LATEST_DIR"

# --- SCP 転送 ---
REMOTE_DESTINATION="${REMOTE_USER}@${REMOTE_IP}:${REMOTE_TARGET_DIR}"
echo -e "\n🚀 Transferring directory to ${CYAN}$REMOTE_DESTINATION${NC}..."

scp -r "$LATEST_DIR" "$REMOTE_DESTINATION"

if [ $? -eq 0 ]; then
    echo -e "\n${GREEN}✅ Directory transferred successfully.${NC}"
else
    echo -e "\n${RED}❌ ERROR: Directory transfer failed.${NC}"
    exit 1
fi
